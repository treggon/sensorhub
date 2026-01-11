
// Livox MID-360 Bridge (SDK2) — Jetson Orin NX ready
// - Receives Livox Ethernet packets via SDK2 callbacks.
// - Multicasts RAW point payloads to 224.1.1.5:56301 and RAW IMU payloads to 224.1.1.5:56401,
//   matching your livox_adapter.py listener.
// - Keeps a small NDJSON diagnostic emitter (UDP to localhost) and control listener.
//
// Environment variables:
//   MID360_CONFIG_PATH     : REQUIRED — path to Livox SDK2 config JSON
//   LIVOX_UDP_PORT         : NDJSON diagnostics UDP port (default: 18080)
//   LIVOX_CTL_PORT         : Control UDP listen port (default: 18181)
//   LIVOX_BRIDGE_STDOUT    : "1" to also print NDJSON lines to stdout (default: off)
//   LIVOX_MC_ADDR          : Multicast group (default: "224.1.1.5")
//   LIVOX_POINTS_PORT      : Multicast points port (default: 56301)
//   LIVOX_IMU_PORT         : Multicast IMU port (default: 56401)
//   LIVOX_MC_TTL           : Multicast TTL (default: 1)
//   LIVOX_MC_LOOP          : Multicast loopback (0/1, default: 1)
//   LIVOX_MC_IFACE         : Outgoing interface IPv4 (optional, default unset)
//
// Build (SDK2 installed to /usr/local):
//   g++ -O2 -std=c++11 livox_bridge.cpp -o livox_bridge \
//       -I/usr/local/include -L/usr/local/lib -llivox_lidar_sdk_shared
//
// Run (example):
//   export MID360_CONFIG_PATH=/home/ubuntu/mid360_config.json
//   ./livox_bridge
//
// Notes:
//  - This bridge expects Livox SDK2 for Mid-360.
//  - You may run a PTP master on your Jetson laser NIC (e.g., `sudo ptp4l -i <nic> -m`)
//    so the Mid-360 will lock timestamps over the wire.
//
// ------------------------------------------------------------------------------

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Livox SDK2 headers (installed by Livox-SDK2)
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

using namespace std::chrono;

// Globals
static std::atomic<bool> g_running(true);

// NDJSON diagnostic UDP emitter (localhost)
static int g_diag_sock = -1;
static sockaddr_in g_diag_dst;
static uint16_t g_diag_port = 18080;
static bool g_emit_stdout = false;

// Control UDP listener (localhost)
static uint16_t g_ctl_port = 18181;

// Device handle tracking
static std::vector<uint32_t> g_handles;
static std::mutex g_handles_mtx;

// Multicast sockets for raw payloads
static int g_mc_pts = -1, g_mc_imu = -1;
static sockaddr_in g_mc_pts_dst, g_mc_imu_dst;

// Multicast config
static std::string g_mc_addr = "224.1.1.5";
static uint16_t g_mc_pts_port = 56301;
static uint16_t g_mc_imu_port = 56401;
static int g_mc_ttl = 1;
static int g_mc_loop = 1; // allow loopback so local adapter can receive
static std::string g_mc_iface; // optional outgoing interface IPv4

// -----------------------------------------------------------------------------
// Helpers
static uint64_t now_us() {
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

static void add_handle(uint32_t h) {
  std::lock_guard<std::mutex> lk(g_handles_mtx);
  for (auto &x : g_handles) if (x == h) return;
  g_handles.push_back(h);
}

static void emit_ndjson(const std::string &line) {
  if (g_diag_sock >= 0) {
    sendto(g_diag_sock, line.c_str(), (int)line.size(), 0,
           (struct sockaddr*)&g_diag_dst, sizeof(g_diag_dst));
  }
  if (g_emit_stdout) {
    std::cout << line << std::endl;
  }
}

// Minimal JSON parsing helpers (replace with rapidjson in production)
static std::string find_str(const std::string& s, const std::string& key) {
  size_t p = s.find(key); if (p == std::string::npos) return "";
  p = s.find(':', p); if (p == std::string::npos) return "";
  size_t q = s.find('\"', p+1); if (q == std::string::npos) return "";
  size_t r = s.find('\"', q+1); if (r == std::string::npos) return "";
  return s.substr(q+1, r-q-1);
}

static int find_int(const std::string& s, const std::string& key, int defv) {
  size_t p = s.find(key); if (p == std::string::npos) return defv;
  p = s.find(':', p); if (p == std::string::npos) return defv;
  char* end = nullptr;
  const char* start = s.c_str() + p + 1;
  long v = std::strtol(start, &end, 10);
  return (end != start) ? (int)v : defv;
}

// -----------------------------------------------------------------------------
// Multicast setup
static bool init_multicast() {
  // Read env overrides
  if (const char* p = std::getenv("LIVOX_MC_ADDR")) g_mc_addr = p;
  if (const char* p = std::getenv("LIVOX_POINTS_PORT")) g_mc_pts_port = (uint16_t)std::atoi(p);
  if (const char* p = std::getenv("LIVOX_IMU_PORT")) g_mc_imu_port = (uint16_t)std::atoi(p);
  if (const char* p = std::getenv("LIVOX_MC_TTL")) g_mc_ttl = std::atoi(p);
  if (const char* p = std::getenv("LIVOX_MC_LOOP")) g_mc_loop = std::atoi(p);
  if (const char* p = std::getenv("LIVOX_MC_IFACE")) g_mc_iface = p;

  g_mc_pts = socket(AF_INET, SOCK_DGRAM, 0);
  g_mc_imu = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_mc_pts < 0 || g_mc_imu < 0) {
    perror("multicast sockets");
    return false;
  }

  // TTL
  if (setsockopt(g_mc_pts, IPPROTO_IP, IP_MULTICAST_TTL, &g_mc_ttl, sizeof(g_mc_ttl)) < 0)
    perror("setsockopt IP_MULTICAST_TTL (points)");
  if (setsockopt(g_mc_imu, IPPROTO_IP, IP_MULTICAST_TTL, &g_mc_ttl, sizeof(g_mc_ttl)) < 0)
    perror("setsockopt IP_MULTICAST_TTL (imu)");

  // Loopback
  if (setsockopt(g_mc_pts, IPPROTO_IP, IP_MULTICAST_LOOP, &g_mc_loop, sizeof(g_mc_loop)) < 0)
    perror("setsockopt IP_MULTICAST_LOOP (points)");
  if (setsockopt(g_mc_imu, IPPROTO_IP, IP_MULTICAST_LOOP, &g_mc_loop, sizeof(g_mc_loop)) < 0)
    perror("setsockopt IP_MULTICAST_LOOP (imu)");

  // Optional outgoing interface
  if (!g_mc_iface.empty()) {
    in_addr iface_addr{};
    iface_addr.s_addr = inet_addr(g_mc_iface.c_str());
    if (iface_addr.s_addr != INADDR_NONE) {
      if (setsockopt(g_mc_pts, IPPROTO_IP, IP_MULTICAST_IF, &iface_addr, sizeof(iface_addr)) < 0)
        perror("setsockopt IP_MULTICAST_IF (points)");
      if (setsockopt(g_mc_imu, IPPROTO_IP, IP_MULTICAST_IF, &iface_addr, sizeof(iface_addr)) < 0)
        perror("setsockopt IP_MULTICAST_IF (imu)");
    }
  }

  // Destination addresses
  std::memset(&g_mc_pts_dst, 0, sizeof(g_mc_pts_dst));
  g_mc_pts_dst.sin_family = AF_INET;
  g_mc_pts_dst.sin_addr.s_addr = inet_addr(g_mc_addr.c_str());
  g_mc_pts_dst.sin_port = htons(g_mc_pts_port);

  std::memset(&g_mc_imu_dst, 0, sizeof(g_mc_imu_dst));
  g_mc_imu_dst.sin_family = AF_INET;
  g_mc_imu_dst.sin_addr.s_addr = inet_addr(g_mc_addr.c_str());
  g_mc_imu_dst.sin_port = htons(g_mc_imu_port);

  return true;
}

// -----------------------------------------------------------------------------
// SDK2 callbacks
static void ControlAckCallback(livox_status status, uint32_t handle,
                               LivoxLidarAsyncControlResponse* resp, void* /*client_data*/) {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "{\"type\":\"ack\",\"status\":%d,\"handle\":%u,\"ret_code\":%u,\"error_key\":%u}",
    (int)status, handle,
    resp ? resp->ret_code : 255,
    resp ? resp->error_key : 0);
  emit_ndjson(buf);
}

static void PointCloudCallback(uint32_t handle, uint8_t /*dev_type*/,
                               LivoxLidarEthernetPacket* pkt, void* /*client_data*/) {
  if (!pkt) return;
  add_handle(handle);

  // Forward RAW point payload to multicast points port
  if (g_mc_pts >= 0 && pkt->length > 0) {
    sendto(g_mc_pts, (const char*)pkt->data, (int)pkt->length, 0,
           (struct sockaddr*)&g_mc_pts_dst, sizeof(g_mc_pts_dst));
  }

  // Optional NDJSON diagnostics
  char meta[256];
  uint64_t ts_us = now_us();
  std::snprintf(meta, sizeof(meta),
    "{\"type\":\"frame\",\"ts_us\":%" PRIu64 ",\"handle\":%u,"
    "\"n_points\":%u,\"data_type\":%u,\"seq\":%u}",
    ts_us, handle, pkt->dot_num, pkt->data_type, pkt->frame_cnt);
  emit_ndjson(meta);
}

static void ImuCallback(uint32_t handle, uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* pkt, void* /*client_data*/) {
  if (!pkt) return;
  add_handle(handle);

  // Forward RAW IMU payload to multicast IMU port
  if (g_mc_imu >= 0 && pkt->length > 0) {
    sendto(g_mc_imu, (const char*)pkt->data, (int)pkt->length, 0,
           (struct sockaddr*)&g_mc_imu_dst, sizeof(g_mc_imu_dst));
  }

  // Optional NDJSON (tiny)
  char meta[128];
  uint64_t ts_us = now_us();
  std::snprintf(meta, sizeof(meta),
    "{\"type\":\"imu_pkt\",\"ts_us\":%" PRIu64 ",\"handle\":%u,\"len\":%u}",
    ts_us, handle, pkt->length);
  emit_ndjson(meta);
}

static void InfoChangeCallback(uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
  add_handle(handle);
  if (!info) return;
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "{\"type\":\"info\",\"handle\":%u,\"dev_type\":%u,\"sn\":\"%.16s\",\"ip\":\"%.16s\"}",
    handle, info->dev_type, info->sn, info->lidar_ip);
  emit_ndjson(buf);
}

// -----------------------------------------------------------------------------
// Apply function to all handles
template <typename Fn>
static void for_each_handle(Fn fn) {
  std::lock_guard<std::mutex> lk(g_handles_mtx);
  for (auto h : g_handles) fn(h);
}

// Control message handler (adapter -> bridge)
static void handle_command(const std::string& msg) {
  const std::string cmd = find_str(msg, "cmd");

  if (cmd == "set_work_mode") {
    const LivoxLidarWorkMode wm = (LivoxLidarWorkMode)find_int(msg, "mode", (int)kLivoxLidarNormal);
    for_each_handle(&{ SetLivoxLidarWorkMode(h, wm, ControlAckCallback, NULL); });
  }
  else if (cmd == "set_pattern_mode") {
    const LivoxLidarScanPattern sp = (LivoxLidarScanPattern)find_int(
      msg, "pattern_mode", (int)kLivoxLidarScanPatternNoneRepetive);
    for_each_handle(&{ SetLivoxLidarScanPattern(h, sp, ControlAckCallback, NULL); });
  }
  else if (cmd == "set_fov") {
    FovCfg cfg;
    cfg.yaw_start   = find_int(msg, "yaw_start", 0);
    cfg.yaw_stop    = find_int(msg, "yaw_stop", 0);
    cfg.pitch_start = find_int(msg, "pitch_start", -7);
    cfg.pitch_stop  = find_int(msg, "pitch_stop", 52);
    cfg.rsvd = 0;
    const int en = find_int(msg, "enable", 1);
    for_each_handle(&{
      SetLivoxLidarFovCfg1(h, &cfg, ControlAckCallback, NULL);
      EnableLivoxLidarFov(h, (uint8_t)en, ControlAckCallback, NULL);
    });
  }
  else if (cmd == "set_imu_enable") {
    const int en = find_int(msg, "enable", 1);
    for_each_handle(&{
      if (en) EnableLivoxLidarImuData(h, ControlAckCallback, NULL);
      else    DisableLivoxLidarImuData(h, ControlAckCallback, NULL);
    });
  }
  else if (cmd == "set_time_sync") {
    // Example: pass RMC sentence, if you use GPS string sync instead of PTP.
    const std::string rmc = find_str(msg, "rmc");
    if (!rmc.empty()) {
      for_each_handle(&{
        SetLivoxLidarRmcSyncTime(h, rmc.c_str(), (uint16_t)rmc.size(), NULL, NULL);
      });
    }
  }
}

// Control listener thread
static void control_thread() {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { std::perror("control socket"); return; }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  addr.sin_port = htons(g_ctl_port);

  if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::perror("control bind");
    close(sock);
    return;
  }

  char buf[4096];
  while (g_running.load()) {
    sockaddr_in src{};
    socklen_t sl = sizeof(src);
    int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr*)&src, &sl);
    if (n > 0) {
      buf[n] = '\0';
      handle_command(std::string(buf));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  close(sock);
}

// Signal handler
static void on_sigint(int) { g_running.store(false); }

// -----------------------------------------------------------------------------
// Main
int main(int /*argc*/, char** /*argv*/) {
  signal(SIGINT, on_sigint);

  // Required config path for SDK2
  const char* cfg_path = std::getenv("MID360_CONFIG_PATH");
  if (!cfg_path || std::strlen(cfg_path) == 0) {
    std::cerr << "MID360_CONFIG_PATH env var is required (SDK2 JSON)." << std::endl;
    return 2;
  }

  // NDJSON diagnostics configuration
  if (const char* p = std::getenv("LIVOX_UDP_PORT")) g_diag_port = (uint16_t)std::atoi(p);
  if (const char* p = std::getenv("LIVOX_CTL_PORT")) g_ctl_port = (uint16_t)std::atoi(p);
  g_emit_stdout = (std::getenv("LIVOX_BRIDGE_STDOUT") &&
                   std::string(std::getenv("LIVOX_BRIDGE_STDOUT")) == "1");

  // NDJSON UDP emitter to localhost
  g_diag_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_diag_sock < 0) { std::perror("diag udp socket"); /*_continue anyway*/ }
  std::memset(&g_diag_dst, 0, sizeof(g_diag_dst));
  g_diag_dst.sin_family = AF_INET;
  g_diag_dst.sin_addr.s_addr = inet_addr("127.0.0.1");
  g_diag_dst.sin_port = htons(g_diag_port);

  // Initialize multicast raw payload forwarding
  if (!init_multicast()) {
    std::cerr << "Multicast setup failed." << std::endl;
    // We can continue for NDJSON only, but adapter expects multicast; warn:
    emit_ndjson("{\"type\":\"warn\",\"msg\":\"multicast_setup_failed\"}");
  }

  // Init SDK2 (host IP inferred from JSON; pass empty string)
  if (!LivoxLidarSdkInit(cfg_path, "", NULL)) {
    std::cerr << "LivoxLidarSdkInit failed." << std::endl;
    return 4;
  }

  // Register callbacks
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, NULL);
  SetLivoxLidarImuDataCallback(ImuCallback, NULL);
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, NULL);

  // Start SDK worker
  if (!LivoxLidarSdkStart()) {
    std::cerr << "LivoxLidarSdkStart failed." << std::endl;
    LivoxLidarSdkUninit();
    return 5;
  }

  // Control listener thread
  std::thread ctl(control_thread);

  // Run until SIGINT
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  LivoxLidarSdkUninit();
  if (ctl.joinable()) ctl.join();

  if (g_diag_sock >= 0) close(g_diag_sock);
  if (g_mc_pts >= 0) close(g_mc_pts);
  if (g_mc_imu >= 0) close(g_mc_imu);

  return 0;
}

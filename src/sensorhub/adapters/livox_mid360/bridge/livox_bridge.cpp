// Livox MID-360 Bridge (SDK2) — NDJSON emitter with extrinsic parsing (no ROS)
// Emits normalized xyzi point chunks and IMU samples to UDP localhost, accepts JSON controls.

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
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Livox SDK2 headers
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

using namespace std::chrono;

// ----------------------------- Globals -----------------------------
static std::atomic<bool> g_running(true);
static int g_udp_sock = -1;
static sockaddr_in g_udp_dst;
static uint16_t g_emit_port = 18080;
static uint16_t g_ctl_port  = 18181;
static bool g_emit_stdout = false;

static std::vector<uint32_t> g_handles;           // observed handles
static std::mutex g_handles_mtx;

struct Extrinsic {
  double roll_deg{0.0}, pitch_deg{0.0}, yaw_deg{0.0};
  double tx{0.0}, ty{0.0}, tz{0.0};                 // meters
};
static std::map<std::string, Extrinsic> g_extrinsics_by_ip; // ip -> extrinsic
static std::map<uint32_t, std::string> g_handle_ip;         // handle -> ip
static std::mutex g_extr_mtx;

// ----------------------------- Utils -----------------------------
static uint64_t now_us() {
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}
static void emit_ndjson(const std::string& line) {
  if (g_udp_sock >= 0) {
    sendto(g_udp_sock, line.c_str(), (int)line.size(), 0, (struct sockaddr*)&g_udp_dst, sizeof(g_udp_dst));
  }
  if (g_emit_stdout) {
    std::cout << line << std::endl;
  }
}
static void add_handle(uint32_t h) {
  std::lock_guard<std::mutex> lk(g_handles_mtx);
  for (size_t i = 0; i < g_handles.size(); ++i) if (g_handles[i] == h) return;
  g_handles.push_back(h);
}

// Minimal JSON helpers for control messages
static std::string find_str(const std::string& s, const std::string& key) {
  size_t p = s.find(key); if (p == std::string::npos) return "";
  p = s.find(':', p); if (p == std::string::npos) return "";
  size_t q = s.find('"', p + 1); if (q == std::string::npos) return "";
  size_t r = s.find('"', q + 1); if (r == std::string::npos) return "";
  return s.substr(q + 1, r - q - 1);
}
static int find_int(const std::string& s, const std::string& key, int defv) {
  size_t p = s.find(key); if (p == std::string::npos) return defv;
  p = s.find(':', p); if (p == std::string::npos) return defv;
  char* end = 0; const char* start = s.c_str() + p + 1;
  long v = std::strtol(start, &end, 10);
  return (end != start) ? (int)v : defv;
}

// ----------------------------- Extrinsic parsing (SDK2 config JSON) -----------------------------
static void parse_extrinsics_from_cfg(const char* cfg_path) {
  std::lock_guard<std::mutex> lk(g_extr_mtx);
  g_extrinsics_by_ip.clear();
  if (!cfg_path) return;
  std::ifstream f(cfg_path);
  if (!f.good()) return;
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  size_t pos = 0;
  while (true) {
    // Find next "ip" field
    size_t ip_key = s.find("\"ip\"", pos);
    if (ip_key == std::string::npos) break;
    size_t colon = s.find(':', ip_key);
    size_t q1 = s.find('"', colon + 1);
    size_t q2 = s.find('"', q1 + 1);
    if (colon == std::string::npos || q1 == std::string::npos || q2 == std::string::npos) break;
    std::string ip = s.substr(q1 + 1, q2 - q1 - 1);

    // Look for extrinsic_parameter block following this ip
    size_t ext = s.find("extrinsic_parameter", q2);
    if (ext == std::string::npos) { pos = q2 + 1; continue; }
    size_t brace = s.find('{', ext);
    size_t endb  = s.find('}', brace);
    if (brace == std::string::npos || endb == std::string::npos) { pos = q2 + 1; continue; }
    std::string blk = s.substr(brace, endb - brace + 1);

    auto find_num = [&](const char* key, double defv) -> double {
      size_t k = blk.find(key);
      if (k == std::string::npos) return defv;
      k = blk.find(':', k);
      if (k == std::string::npos) return defv;
      char* end = 0; const char* start = blk.c_str() + (k + 1);
      double v = std::strtod(start, &end);
      return (end != start) ? v : defv;
    };

    Extrinsic ex; // defaults
    ex.roll_deg  = find_num("roll", 0.0);
    ex.pitch_deg = find_num("pitch", 0.0);
    ex.yaw_deg   = find_num("yaw", 0.0);
    ex.tx        = find_num("x", 0.0); // meters assumed
    ex.ty        = find_num("y", 0.0);
    ex.tz        = find_num("z", 0.0);

    g_extrinsics_by_ip[ip] = ex;
    pos = endb + 1;
  }
}

// Build rotation matrix (ZYX yaw-pitch-roll)
struct RotM {
  double R00, R01, R02;
  double R10, R11, R12;
  double R20, R21, R22;
};
static RotM make_rotm(const Extrinsic& ex) {
  const double cr = std::cos(ex.roll_deg  * M_PI/180.0);
  const double sr = std::sin(ex.roll_deg  * M_PI/180.0);
  const double cp = std::cos(ex.pitch_deg * M_PI/180.0);
  const double sp = std::sin(ex.pitch_deg * M_PI/180.0);
  const double cy = std::cos(ex.yaw_deg   * M_PI/180.0);
  const double sy = std::sin(ex.yaw_deg   * M_PI/180.0);
  RotM M;
  M.R00 = cy*cp;               M.R01 = cy*sp*sr - sy*cr; M.R02 = cy*sp*cr + sy*sr;
  M.R10 = sy*cp;               M.R11 = sy*sp*sr + cy*cr; M.R12 = sy*sp*cr - cy*sr;
  M.R20 = -sp;                 M.R21 = cp*sr;            M.R22 = cp*cr;
  return M;
}

// ----------------------------- Control ack -----------------------------
static void ControlAckCallback(livox_status status, uint32_t handle,
                               LivoxLidarAsyncControlResponse* resp, void* /*client_data*/) {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "{\"type\":\"ack\",\"status\":%d,\"handle\":%u,\"ret_code\":%u,\"error_key\":%u}",
    (int)status, handle, resp ? resp->ret_code : 255, resp ? resp->error_key : 0);
  emit_ndjson(buf);
}

// ----------------------------- Callbacks -----------------------------
static void PointCloudCallback(const uint32_t handle, const uint8_t /*dev_type*/,
                               LivoxLidarEthernetPacket* pkt, void* /*client_data*/) {
  if (!pkt) return;
  add_handle(handle);
  const uint8_t* data = reinterpret_cast<const uint8_t*>(pkt->data);
  const uint32_t n = pkt->dot_num;
  const uint8_t dt = pkt->data_type;
  const uint32_t seq = pkt->frame_cnt;
  const uint64_t ts_us = now_us();

  // Lookup extrinsic for this handle via IP
  Extrinsic ex; RotM M;
  {
    std::lock_guard<std::mutex> lk(g_extr_mtx);
    auto itH = g_handle_ip.find(handle);
    if (itH != g_handle_ip.end()) {
      auto itE = g_extrinsics_by_ip.find(itH->second);
      if (itE != g_extrinsics_by_ip.end()) {
        ex = itE->second;
      }
    }
    M = make_rotm(ex);
  }
  auto apply_extrinsic = [&](double& x, double& y, double& z) {
    double X = M.R00*x + M.R01*y + M.R02*z + ex.tx;
    double Y = M.R10*x + M.R11*y + M.R12*z + ex.ty;
    double Z = M.R20*x + M.R21*y + M.R22*z + ex.tz;
    x = X; y = Y; z = Z;
  };

  const char* dt_name = (dt == kLivoxLidarCartesianCoordinateHighData) ? "cartesian_high" :
                        (dt == kLivoxLidarCartesianCoordinateLowData)  ? "cartesian_low"  :
                        (dt == kLivoxLidarSphericalCoordinateData)     ? "spherical"      : "unknown";
  char hbuf[256];
  std::snprintf(hbuf, sizeof(hbuf),
    "{\"type\":\"points\",\"ts_us\":%" PRIu64 ",\"handle\":%u,\"seq\":%u,\"fields\":\"xyzi\",\"data_type\":\"%s\",\"extrinsic_applied\":true,\"points\":[",
    ts_us, handle, seq, dt_name);
  std::string hdr(hbuf);

  const uint32_t CHUNK = 2000;
  uint32_t emitted = 0;
  while (emitted < n) {
    std::string line = hdr;
    uint32_t take = std::min(CHUNK, n - emitted);

    if (dt == kLivoxLidarCartesianCoordinateHighData) {
      struct LivoxLidarCartesianHighRawPoint { int32_t x; int32_t y; int32_t z; uint8_t reflectivity; uint8_t tag; };
      auto pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data + emitted * sizeof(LivoxLidarCartesianHighRawPoint));
      for (uint32_t i = 0; i < take; ++i) {
        double x = pts[i].x / 1000.0, y = pts[i].y / 1000.0, z = pts[i].z / 1000.0; // mm->m
        apply_extrinsic(x,y,z);
        char pb[128];
        std::snprintf(pb, sizeof(pb), "[%.5f,%.5f,%.5f,%u]%s", x, y, z, (unsigned)pts[i].reflectivity, (i+1<take)?",":"");
        line += pb;
      }
    } else if (dt == kLivoxLidarCartesianCoordinateLowData) {
      struct LivoxLidarCartesianLowRawPoint { int16_t x; int16_t y; int16_t z; uint8_t reflectivity; uint8_t tag; };
      auto pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data + emitted * sizeof(LivoxLidarCartesianLowRawPoint));
      for (uint32_t i = 0; i < take; ++i) {
        double x = pts[i].x / 100.0, y = pts[i].y / 100.0, z = pts[i].z / 100.0; // cm->m
        apply_extrinsic(x,y,z);
        char pb[128];
        std::snprintf(pb, sizeof(pb), "[%.5f,%.5f,%.5f,%u]%s", x, y, z, (unsigned)pts[i].reflectivity, (i+1<take)?",":"");
        line += pb;
      }
    } else if (dt == kLivoxLidarSphericalCoordinateData) {
      struct LivoxLidarSphericalRawPoint { int16_t depth_mm; int16_t theta_cdeg; int16_t phi_cdeg; uint8_t reflectivity; uint8_t tag; };
      auto pts = reinterpret_cast<const LivoxLidarSphericalRawPoint*>(data + emitted * sizeof(LivoxLidarSphericalRawPoint));
      for (uint32_t i = 0; i < take; ++i) {
        const double r   = pts[i].depth_mm / 1000.0;                                    // mm->m
        const double th  = (pts[i].theta_cdeg / 100.0) * (M_PI / 180.0);                 // centideg->rad
        const double phi = (pts[i].phi_cdeg   / 100.0) * (M_PI / 180.0);
        double x = r * std::cos(phi) * std::cos(th);
        double y = r * std::cos(phi) * std::sin(th);
        double z = r * std::sin(phi);
        apply_extrinsic(x,y,z);
        char pb[128];
        std::snprintf(pb, sizeof(pb), "[%.5f,%.5f,%.5f,%u]%s", x, y, z, (unsigned)pts[i].reflectivity, (i+1<take)?",":"");
        line += pb;
      }
    } else {
      // Unknown type; skip
      break;
    }

    line += "]}";
    emit_ndjson(line);
    emitted += take;
  }
}

static void ImuCallback(const uint32_t handle, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* pkt, void* /*client_data*/) {
  if (!pkt) return;
  add_handle(handle);
  if (pkt->length >= sizeof(LivoxLidarImuRawPoint)) {
    const LivoxLidarImuRawPoint* imu = reinterpret_cast<const LivoxLidarImuRawPoint*>(pkt->data);
    uint64_t ts_us = now_us();
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "{\"type\":\"imu\",\"ts_us\":%" PRIu64 ",\"handle\":%u,\"ax\":%.6f,\"ay\":%.6f,\"az\":%.6f,\"gx\":%.6f,\"gy\":%.6f,\"gz\":%.6f}",
      ts_us, handle, imu->acc_x, imu->acc_y, imu->acc_z, imu->gyro_x, imu->gyro_y, imu->gyro_z);
    emit_ndjson(buf);
  }
}

static void InfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
  add_handle(handle);
  if (!info) return;
  {
    std::lock_guard<std::mutex> lk(g_extr_mtx);
    g_handle_ip[handle] = std::string(info->lidar_ip);
  }
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "{\"type\":\"info\",\"handle\":%u,\"dev_type\":%u,\"sn\":\"%.16s\",\"ip\":\"%.16s\"}",
    handle, info->dev_type, info->sn, info->lidar_ip);
  emit_ndjson(buf);
}

// ----------------------------- Control message handler -----------------------------
static void handle_command(const std::string& msg) {
  const std::string cmd = find_str(msg, "cmd");
  if (cmd == "set_work_mode") {
    const LivoxLidarWorkMode wm = (LivoxLidarWorkMode)find_int(msg, "mode", (int)kLivoxLidarNormal);
    std::lock_guard<std::mutex> lk(g_handles_mtx);
    for (auto h : g_handles) SetLivoxLidarWorkMode(h, wm, ControlAckCallback, NULL);
  }
  else if (cmd == "set_pattern_mode") {
    const LivoxLidarScanPattern sp = (LivoxLidarScanPattern)find_int(msg, "pattern_mode", (int)kLivoxLidarScanPatternNoneRepetive);
    std::lock_guard<std::mutex> lk(g_handles_mtx);
    for (auto h : g_handles) SetLivoxLidarScanPattern(h, sp, ControlAckCallback, NULL);
  }
  else if (cmd == "set_fov") {
    FovCfg cfg; cfg.yaw_start = find_int(msg, "yaw_start", 0); cfg.yaw_stop = find_int(msg, "yaw_stop", 0);
    cfg.pitch_start = find_int(msg, "pitch_start", -7); cfg.pitch_stop = find_int(msg, "pitch_stop", 52); cfg.rsvd = 0;
    const int en = find_int(msg, "enable", 1);
    std::lock_guard<std::mutex> lk(g_handles_mtx);
    for (auto h : g_handles) { SetLivoxLidarFovCfg1(h, &cfg, ControlAckCallback, NULL); EnableLivoxLidarFov(h, (uint8_t)en, ControlAckCallback, NULL); }
  }
  else if (cmd == "set_imu_enable") {
    const int en = find_int(msg, "enable", 1);
    std::lock_guard<std::mutex> lk(g_handles_mtx);
    for (auto h : g_handles) {
      if (en) EnableLivoxLidarImuData(h, ControlAckCallback, NULL); else DisableLivoxLidarImuData(h, ControlAckCallback, NULL);
    }
  }
  else if (cmd == "set_time_sync") {
    const std::string rmc = find_str(msg, "rmc");
    if (!rmc.empty()) {
      std::lock_guard<std::mutex> lk(g_handles_mtx);
      for (auto h : g_handles) SetLivoxLidarRmcSyncTime(h, rmc.c_str(), (uint16_t)rmc.size(), NULL, NULL);
    }
  }
  else if (cmd == "set_extrinsic") {
    // Update extrinsic for a specific ip or for all known ips
    const std::string ip   = find_str(msg, "ip");
    double roll  = (double)find_int(msg, "roll", 0);
    double pitch = (double)find_int(msg, "pitch", 0);
    double yaw   = (double)find_int(msg, "yaw", 0);
    auto find_double = [&](const std::string& key, double defv) -> double {
      size_t p = msg.find(key); if (p == std::string::npos) return defv; p = msg.find(':', p); if (p == std::string::npos) return defv;
      char* end = 0; const char* start = msg.c_str() + p + 1; double v = std::strtod(start, &end); return (end != start)?v:defv; };
    double tx = find_double("tx", 0.0); double ty = find_double("ty", 0.0); double tz = find_double("tz", 0.0);
    Extrinsic ex; ex.roll_deg = roll; ex.pitch_deg = pitch; ex.yaw_deg = yaw; ex.tx = tx; ex.ty = ty; ex.tz = tz;
    {
      std::lock_guard<std::mutex> lk(g_extr_mtx);
      if (!ip.empty()) g_extrinsics_by_ip[ip] = ex; else {
        for (auto& kv : g_extrinsics_by_ip) kv.second = ex;
      }
    }
    emit_ndjson("{\"type\":\"ack\",\"cmd\":\"set_extrinsic\",\"status\":0}");
  }
}

static void control_thread() {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { std::perror("control socket"); return; }
  sockaddr_in addr; std::memset(&addr, 0, sizeof(addr)); addr.sin_family = AF_INET; addr.sin_addr.s_addr = inet_addr("127.0.0.1"); addr.sin_port = htons(g_ctl_port);
  if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { std::perror("control bind"); close(sock); return; }
  char buf[4096];
  while (g_running.load()) {
    sockaddr_in src; socklen_t sl = sizeof(src);
    int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr*)&src, &sl);
    if (n > 0) { buf[n] = '\0'; handle_command(std::string(buf)); }
    else { std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  }
  close(sock);
}

static void on_sigint(int) { g_running.store(false); }

int main(int /*argc*/, char** /*argv*/) {
  signal(SIGINT, on_sigint);
  const char* cfg_path = std::getenv("MID360_CONFIG_PATH");
  if (!cfg_path || std::strlen(cfg_path) == 0) {
    std::cerr << "MID360_CONFIG_PATH env var is required (SDK2 JSON)." << std::endl;
    return 2;
  }
  if (const char* p = std::getenv("LIVOX_UDP_PORT")) g_emit_port = (uint16_t)std::atoi(p);
  if (const char* p = std::getenv("LIVOX_CTL_PORT")) g_ctl_port  = (uint16_t)std::atoi(p);
  g_emit_stdout = (std::getenv("LIVOX_BRIDGE_STDOUT") && std::string(std::getenv("LIVOX_BRIDGE_STDOUT")) == "1");

  // UDP emitter
  g_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_udp_sock < 0) { std::perror("udp socket"); return 3; }
  std::memset(&g_udp_dst, 0, sizeof(g_udp_dst)); g_udp_dst.sin_family = AF_INET; g_udp_dst.sin_addr.s_addr = inet_addr("127.0.0.1"); g_udp_dst.sin_port = htons(g_emit_port);

  // Parse extrinsics from SDK2 config JSON (maps ip -> extrinsic)
  parse_extrinsics_from_cfg(cfg_path);

  // Init SDK2 (host_ip inferred from JSON; pass "")
  if (!LivoxLidarSdkInit(cfg_path, "", NULL)) { std::cerr << "LivoxLidarSdkInit failed." << std::endl; return 4; }
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, NULL);
  SetLivoxLidarImuDataCallback(ImuCallback, NULL);
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, NULL);
  if (!LivoxLidarSdkStart()) { std::cerr << "LivoxLidarSdkStart failed." << std::endl; LivoxLidarSdkUninit(); return 5; }

  std::thread ctl(control_thread);
  while (g_running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  LivoxLidarSdkUninit();
  if (ctl.joinable()) ctl.join();
  if (g_udp_sock >= 0) close(g_udp_sock);
  return 0;
}
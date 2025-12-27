
// livox_bridge.cpp (multi-device JSON-aware, SDK2-corrected)
// Emits NDJSON frames tagged with "lidar_id" for Livox Mid-360 units.
//
// Config file (JSON) expected at: src/sensorhub/config/mid360_config.json
// Optional: set MID360_CONFIG_PATH env to override.
//
// References:
//  - SDK2 quick-start sample (callback signatures & function names)
//    https://github.com/Livox-SDK/Livox-SDK2/tree/master/samples/livox_lidar_quick_start  (see main.cpp)
//  - SDK2 headers (names, types): livox_lidar_api.h, livox_lidar_def.h
//
// NOTE: In SDK2, IMU data arrives via LivoxLidarEthernetPacket*, same as point cloud.
//       The sample prints the packet header (dot_num, data_type, length, frame_cnt).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// --- SDK2 headers (match the sample) ---
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifndef NO_JSON
  #include <nlohmann/json.hpp>
  using json = nlohmann::json;
#endif

namespace {

// --------------------- Config structures ---------------------
struct DevCfg {
  std::string id;
  std::string lidar_ip;
  std::string host_ip;
  int cmd   = 56000;
  int point = 56301; // match Viewer "Point Cloud Port"
  int imu   = 58000;
  int ndjson_port = -1; // -1 => use global bridge port
};

struct BridgeCfg {
  bool stdout_mode = false;
  int  global_port = 18080;
};

// --------------------- UDP sender ---------------------
class UdpSender {
 public:
  explicit UdpSender(int port) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family      = AF_INET;
    addr_.sin_port        = htons(port);
    addr_.sin_addr.s_addr = inet_addr("127.0.0.1");
  }
  ~UdpSender() { if (sock_ >= 0) close(sock_); }
  void send(const std::string& s) {
    if (sock_ >= 0) {
      sendto(sock_, s.data(), s.size(), 0,
             reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
    }
  }
 private:
  int sock_;
  sockaddr_in addr_{};
};

// --------------------- Global state ---------------------
std::vector<DevCfg> g_devices;
BridgeCfg g_bridge_cfg;
bool g_stdout = true;
std::mutex g_mu;

std::unordered_map<std::string, std::unique_ptr<UdpSender>> g_senders;  // device id -> sender
std::unordered_map<std::string, std::string> g_ip_to_id;                // lidar_ip -> id
std::unordered_map<uint32_t, std::string> g_handle_to_id;               // handle -> id

// --------------------- Helpers ---------------------
std::string json_escape(const std::string& in) {
  std::string out; out.reserve(in.size()+8);
  for (char c : in) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '\"': out += "\\\""; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:   out += c;      break;
    }
  }
  return out;
}

void emit_json_line_for_id(const std::string& id, const std::string& line) {
  std::lock_guard<std::mutex> lk(g_mu);
  if (g_stdout) {
    std::cout << line << "\n" << std::flush;
  } else {
    auto it = g_senders.find(id);
    if (it != g_senders.end() && it->second) it->second->send(line);
    else {
      auto git = g_senders.find("__global__");
      if (git != g_senders.end() && git->second) git->second->send(line);
    }
  }
}

// --------------------- Config loading ---------------------
bool load_config(const std::string& path) {
#ifndef NO_JSON
  try {
    std::ifstream ifs(path);
    if (!ifs.good()) {
      std::cerr << "Config not found at " << path
                << ", proceeding with defaults/env\n";
      return false;
    }
    json j; ifs >> j;

    g_devices.clear(); g_ip_to_id.clear();
    if (j.contains("lidars") && j["lidars"].is_array()) {
      for (const auto& l : j["lidars"]) {
        DevCfg d;
        d.id        = l.value("id", "lidar_" + std::to_string(int(g_devices.size())));
        d.lidar_ip  = l.value("lidar_ip", "10.0.0.10");
        d.host_ip   = l.value("host_ip",  "0.0.0.0");
        d.cmd       = l.value("cmd_data_port",   56000);
        d.point     = l.value("point_data_port", 56301);
        d.imu       = l.value("imu_data_port",   58000);
        d.ndjson_port = l.value("ndjson_udp_port", -1);
        g_devices.push_back(d);
        g_ip_to_id[d.lidar_ip] = d.id;
      }
    }
    if (j.contains("bridge")) {
      const auto& b = j["bridge"];
      g_bridge_cfg.global_port = b.value("ndjson_udp_port", 18080);
      g_bridge_cfg.stdout_mode = b.value("stdout", false);
    }
    return true;
  } catch (const std::exception& e) {
    std::cerr << "JSON parse error: " << e.what() << "\n";
    return false;
  }
#else
  (void)path;
  // NO_JSON fallback: single device from environment
  DevCfg d;
  d.id       = std::getenv("LIVOX_ID")       ? std::getenv("LIVOX_ID")       : "mid360_default";
  d.lidar_ip = std::getenv("LIVOX_LIDAR_IP") ? std::getenv("LIVOX_LIDAR_IP") : "10.0.0.10";
  d.host_ip  = std::getenv("LIVOX_HOST_IP")  ? std::getenv("LIVOX_HOST_IP")  : "0.0.0.0";
  d.cmd = 56000; d.point = 56301; d.imu = 58000; d.ndjson_port = -1;
  g_devices = {d};
  g_ip_to_id[d.lidar_ip] = d.id;
  g_bridge_cfg.global_port = std::getenv("LIVOX_UDP_PORT")
                             ? std::stoi(std::getenv("LIVOX_UDP_PORT"))
                             : 18080;
  g_bridge_cfg.stdout_mode = false;
  return true;
#endif
}

void init_senders() {
  g_senders.clear();
  if (!g_bridge_cfg.stdout_mode) {
    g_senders["__global__"] = std::make_unique<UdpSender>(g_bridge_cfg.global_port);
    for (const auto& d : g_devices) {
      if (d.ndjson_port > 0) {
        g_senders[d.id] = std::make_unique<UdpSender>(d.ndjson_port);
      }
    }
  }
}

// --------------------- SDK2 callbacks ---------------------

// Optional "push message" callback (SDK2 provides it)
void LivoxLidarPushMsgCallback(const uint32_t handle,
                               const uint8_t dev_type,
                               const char* info,
                               void* /*client_data*/) {
  // Derive ip string from handle (like the sample)
  in_addr tmp_addr; tmp_addr.s_addr = handle;
  std::string ip = inet_ntoa(tmp_addr);
  auto it = g_ip_to_id.find(ip);
  std::string id = (it != g_ip_to_id.end()) ? it->second : ip;

  std::ostringstream oss;
  oss << '{'
      << "\"type\":\"push\","
      << "\"lidar_id\":\"" << json_escape(id) << "\","
      << "\"handle\":" << handle << ','
      << "\"dev_type\":" << int(dev_type) << ','
      << "\"msg\":\"" << (info ? json_escape(std::string(info)) : "") << "\""
      << '}';
  emit_json_line_for_id(id, oss.str());
}

// Correct SDK2 signature: handle is the first argument; info contains e.g., SN.
void LidarInfoChangeCallback(const uint32_t handle,
                             const LivoxLidarInfo* info,
                             void* /*client_data*/) {
  if (!info) {
    std::cerr << "LidarInfoChangeCallback: info==nullptr\n";
    return;
  }
  // Resolve id via handle -> ip -> id
  in_addr tmp_addr; tmp_addr.s_addr = handle;
  std::string ip = inet_ntoa(tmp_addr);
  auto it = g_ip_to_id.find(ip);
  if (it != g_ip_to_id.end()) {
    g_handle_to_id[handle] = it->second;
  }
  std::string id = g_handle_to_id.count(handle) ? g_handle_to_id[handle] : ip;

  // Emit a compact "info" message; include SN for debugging
  std::ostringstream oss;
  oss << '{'
      << "\"type\":\"info\","
      << "\"lidar_id\":\"" << json_escape(id) << "\","
      << "\"handle\":" << handle << ','
      << "\"sn\":\"" << json_escape(info->sn) << "\""
      << '}';
  emit_json_line_for_id(id, oss.str());

  // Optional: set work mode to NORMAL as in the sample (can pass nullptr for callback)
  // SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, /*WorkModeCallback*/ nullptr, nullptr);
}

// SDK2 IMU callback uses LivoxLidarEthernetPacket* (same as point cloud).
void ImuDataCallback(uint32_t handle,
                     const uint8_t dev_type,
                     LivoxLidarEthernetPacket* data,
                     void* /*client_data*/) {
  if (!data) return;
  auto now = std::chrono::steady_clock::now().time_since_epoch();
  auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  std::string id = g_handle_to_id.count(handle) ? g_handle_to_id[handle] : std::to_string(handle);

  // Emit just the packet header (matching what the sample prints)
  std::ostringstream oss;
  oss << '{'
      << "\"type\":\"imu\","
      << "\"lidar_id\":\"" << json_escape(id) << "\","
      << "\"ts\":" << ms << ','
      << "\"handle\":" << handle << ','
      << "\"dev_type\":" << int(dev_type) << ','
      << "\"dot_num\":" << int(data->dot_num) << ','
      << "\"data_type\":" << int(data->data_type) << ','
      << "\"length\":" << int(data->length) << ','
      << "\"frame_cnt\":" << int(data->frame_cnt)
      << '}';
  emit_json_line_for_id(id, oss.str());
}

// Point cloud callback (matches SDK2 sample)
void PointCloudCallback(uint32_t handle,
                        const uint8_t dev_type,
                        LivoxLidarEthernetPacket* data,
                        void* /*client_data*/) {
  if (!data) return;

  auto now = std::chrono::steady_clock::now().time_since_epoch();
  auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  std::string id = g_handle_to_id.count(handle) ? g_handle_to_id[handle] : std::to_string(handle);

  std::ostringstream oss;
  oss << '{'
      << "\"type\":\"frame\","
      << "\"lidar_id\":\"" << json_escape(id) << "\","
      << "\"ts\":" << ms << ','
      << "\"handle\":" << handle << ','
      << "\"dev_type\":" << int(dev_type) << ','
      << "\"dot_num\":" << int(data->dot_num) << ','
      << "\"data_type\":" << int(data->data_type) << ','
      << "\"length\":" << int(data->length) << ','
      << "\"frame_cnt\":" << int(data->frame_cnt) << ','
      << "\"points\":[";

  // Emit up to the first 64 points (keep payloads light)
  int max_emit = std::min<int>(data->dot_num, 64);
  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    auto* p = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);
    for (int i = 0; i < max_emit; ++i) {
      if (i) oss << ',';
      oss << '{' << "\"x\":" << p[i].x
          << ",\"y\":" << p[i].y
          << ",\"z\":" << p[i].z
          << ",\"reflect\":" << int(p[i].reflectivity) << '}';
    }
  } else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    auto* p = reinterpret_cast<LivoxLidarCartesianLowRawPoint*>(data->data);
    for (int i = 0; i < max_emit; ++i) {
      if (i) oss << ',';
      oss << '{' << "\"x\":" << p[i].x
          << ",\"y\":" << p[i].y
          << ",\"z\":" << p[i].z
          << ",\"reflect\":" << int(p[i].reflectivity) << '}';
    }
  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    auto* p = reinterpret_cast<LivoxLidarSpherPoint*>(data->data);
    for (int i = 0; i < max_emit; ++i) {
      if (i) oss << ',';
      oss << '{' << "\"depth\":" << p[i].depth
          << ",\"theta\":" << p[i].theta
          << ",\"phi\":" << p[i].phi
          << ",\"reflect\":" << int(p[i].reflectivity) << '}';
    }
  }
  oss << "]}";
  emit_json_line_for_id(id, oss.str());
}

} // namespace

// --------------------- main ---------------------
int main(int /*argc*/, char** /*argv*/) {
  // Default path; allow override via env MID360_CONFIG_PATH
  std::string default_cfg = "src/sensorhub/config/mid360_config.json";
  const char* env_cfg = std::getenv("MID360_CONFIG_PATH");
  std::string cfg_path = env_cfg ? env_cfg : default_cfg;

  // Load config (JSON or env fallback)
  load_config(cfg_path);
  g_stdout = g_bridge_cfg.stdout_mode;
  init_senders();

  // Initialize SDK2 with the config path (matches sample usage)
  if (!LivoxLidarSdkInit(cfg_path.c_str())) {
    std::cerr << "Failed to init Livox SDK2\n";
    LivoxLidarSdkUninit();
    return 1;
  }

  // Register callbacks (names/signatures per SDK2 sample)
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);     // point cloud
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);           // imu
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);    // optional push messages
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);// device info changes

  std::cerr << "livox_bridge running (" << (g_stdout ? "stdout" : "udp")
            << ": global=" << g_bridge_cfg.global_port << ") with "
            << g_devices.size() << " device config(s)\n";

  // Run until interrupted
  while (true) { ::usleep(10000); }

  LivoxLidarSdkUninit();
  return 0;
}

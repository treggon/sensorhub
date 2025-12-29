// Livox MID-360 Bridge (SDK2) — Jetson Orin ready (C++11-safe)
// Emits NDJSON frames (pointcloud + IMU) to UDP localhost and listens for control commands.
//
// Environment variables:
//   MID360_CONFIG_PATH : path to SDK2 config JSON (lidar_type: 8)
//   LIVOX_UDP_PORT     : UDP port to emit NDJSON frames (default 18080)
//   LIVOX_CTL_PORT     : UDP port to receive JSON control commands (default 18181)
//   LIVOX_BRIDGE_STDOUT: if "1", also print NDJSON to stdout

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

// Livox SDK2 headers (your copies)
#include "livox_lidar_api.h"   // SDK entry points & controls
#include "livox_lidar_def.h"   // types, enums, packet structs

using namespace std::chrono;

static std::atomic<bool> g_running(true);
static int g_udp_sock = -1;
static sockaddr_in g_udp_dst;
static uint16_t g_emit_port = 18080;
static uint16_t g_ctl_port = 18181;
static bool g_emit_stdout = false;

static std::vector<uint32_t> g_handles;   // device handles observed via callbacks
static std::mutex g_handles_mtx;

static uint64_t now_us() {
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

static void emit_ndjson(const std::string& line) {
    if (g_udp_sock >= 0) {
        sendto(g_udp_sock, line.c_str(), (int)line.size(), 0,
            (struct sockaddr*)&g_udp_dst, sizeof(g_udp_dst));
    }
    if (g_emit_stdout) {
        std::cout << line << std::endl;
    }
}

static void add_handle(uint32_t h) {
    std::lock_guard<std::mutex> lk(g_handles_mtx);
    for (size_t i = 0; i < g_handles.size(); ++i)
        if (g_handles[i] == h) return;
    g_handles.push_back(h);
}

// ---- SDK2 ack callback ----
static void ControlAckCallback(livox_status status, uint32_t handle,
    LivoxLidarAsyncControlResponse* resp, void* /*client_data*/) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "{\"type\":\"ack\",\"status\":%d,\"handle\":%u,\"ret_code\":%u,\"error_key\":%u}",
        (int)status, handle, resp ? resp->ret_code : 255, resp ? resp->error_key : 0);
    emit_ndjson(buf);
}

// ---- Point cloud callback (Ethernet packet) ----
static void PointCloudCallback(const uint32_t handle, const uint8_t /*dev_type*/,
    LivoxLidarEthernetPacket* pkt, void* /*client_data*/) {
    if (!pkt) return;
    add_handle(handle);
    char buf[256];
    uint64_t ts_us = now_us();
    std::snprintf(buf, sizeof(buf),
        "{\"type\":\"frame\",\"ts_us\":%" PRIu64 ",\"handle\":%u,\"n_points\":%u,"
        "\"data_type\":%u,\"seq\":%u}",
        ts_us, handle, pkt->dot_num, pkt->data_type, pkt->frame_cnt);
    emit_ndjson(buf);
}

// ---- IMU callback (Ethernet packet carrying LivoxLidarImuRawPoint) ----
static void ImuCallback(const uint32_t handle, const uint8_t /*dev_type*/,
    LivoxLidarEthernetPacket* pkt, void* /*client_data*/) {
    if (!pkt) return;
    add_handle(handle);
    if (pkt->length >= sizeof(LivoxLidarImuRawPoint)) {
        const LivoxLidarImuRawPoint* imu =
            reinterpret_cast<const LivoxLidarImuRawPoint*>(pkt->data);
        char buf[256];
        uint64_t ts_us = now_us();
        std::snprintf(buf, sizeof(buf),
            "{\"type\":\"imu\",\"ts_us\":%" PRIu64 ",\"handle\":%u,"
            "\"ax\":%.6f,\"ay\":%.6f,\"az\":%.6f,\"gx\":%.6f,\"gy\":%.6f,\"gz\":%.6f}",
            ts_us, handle, imu->acc_x, imu->acc_y, imu->acc_z,
            imu->gyro_x, imu->gyro_y, imu->gyro_z);
        emit_ndjson(buf);
    }
}

// ---- Info change callback (dev_type/SN/IP) ----
static void InfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
    add_handle(handle);
    if (!info) return;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "{\"type\":\"info\",\"handle\":%u,\"dev_type\":%u,\"sn\":\"%.*s\",\"ip\":\"%.*s\"}",
        handle, info->dev_type, 16, info->sn, 16, info->lidar_ip);
    emit_ndjson(buf);
}

// ---- Minimal JSON helpers (replace with rapidjson in prod) ----
static std::string find_str(const std::string& s, const std::string& key) {
    size_t p = s.find(key); if (p == std::string::npos) return "";
    p = s.find(':', p);     if (p == std::string::npos) return "";
    size_t q = s.find('\"', p + 1); if (q == std::string::npos) return "";
    size_t r = s.find('\"', q + 1); if (r == std::string::npos) return "";
    return s.substr(q + 1, r - q - 1);
}

static int find_int(const std::string& s, const std::string& key, int defv) {
    size_t p = s.find(key); if (p == std::string::npos) return defv;
    p = s.find(':', p);     if (p == std::string::npos) return defv;
    char* end = 0; const char* start = s.c_str() + p + 1;
    long v = std::strtol(start, &end, 10);
    return (end != start) ? (int)v : defv;
}

// ---- Apply function to all handles (C++11-safe) ----
template <typename Fn>
static void for_each_handle(Fn fn) {
    std::lock_guard<std::mutex> lk(g_handles_mtx);
    for (size_t i = 0; i < g_handles.size(); ++i) {
        fn(g_handles[i]);
    }
}

// ---- Control message handler (adapter -> bridge) ----
static void handle_command(const std::string& msg) {
    const std::string cmd = find_str(msg, "cmd");

    if (cmd == "set_work_mode") {
        const LivoxLidarWorkMode wm =
            (LivoxLidarWorkMode)find_int(msg, "mode", (int)kLivoxLidarNormal);
        for_each_handle([&](uint32_t h) {
            SetLivoxLidarWorkMode(h, wm, ControlAckCallback, NULL);
            });

    }
    else if (cmd == "set_pattern_mode") {
        const LivoxLidarScanPattern sp =
            (LivoxLidarScanPattern)find_int(msg, "pattern_mode",
                (int)kLivoxLidarScanPatternNoneRepetive);
        for_each_handle([&](uint32_t h) {
            SetLivoxLidarScanPattern(h, sp, ControlAckCallback, NULL);
            });

    }
    else if (cmd == "set_fov") {
        FovCfg cfg;
        cfg.yaw_start = find_int(msg, "yaw_start", 0);
        cfg.yaw_stop = find_int(msg, "yaw_stop", 0);
        cfg.pitch_start = find_int(msg, "pitch_start", -7);
        cfg.pitch_stop = find_int(msg, "pitch_stop", 52);
        cfg.rsvd = 0;
        const int en = find_int(msg, "enable", 1);
        for_each_handle([&](uint32_t h) {
            SetLivoxLidarFovCfg1(h, &cfg, ControlAckCallback, NULL);
            EnableLivoxLidarFov(h, (uint8_t)en, ControlAckCallback, NULL);
            });

    }
    else if (cmd == "set_imu_enable") {
        const int en = find_int(msg, "enable", 1);
        for_each_handle([&](uint32_t h) {
            if (en) EnableLivoxLidarImuData(h, ControlAckCallback, NULL);
            else    DisableLivoxLidarImuData(h, ControlAckCallback, NULL);
            });

    }
    else if (cmd == "set_time_sync") {
        const std::string rmc = find_str(msg, "rmc");
        if (!rmc.empty()) {
            for_each_handle([&](uint32_t h) {
                SetLivoxLidarRmcSyncTime(h, rmc.c_str(), (uint16_t)rmc.size(), NULL, NULL);
                });
        }
    }
}

// ---- Control listener thread ----
static void control_thread() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { std::perror("control socket"); return; }

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
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
        sockaddr_in src; socklen_t sl = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr*)&src, &sl);
        if (n > 0) {
            buf[n] = '\0';
            handle_command(std::string(buf));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    close(sock);
}

// ---- Signal ----
static void on_sigint(int) { g_running.store(false); }

// ---- Main ----
int main(int /*argc*/, char** /*argv*/) {
    signal(SIGINT, on_sigint);

    const char* cfg_path = std::getenv("MID360_CONFIG_PATH");
    if (!cfg_path || std::strlen(cfg_path) == 0) {
        std::cerr << "MID360_CONFIG_PATH env var is required (SDK2 JSON)." << std::endl;
        return 2;
    }
    if (const char* p = std::getenv("LIVOX_UDP_PORT")) g_emit_port = (uint16_t)std::atoi(p);
    if (const char* p = std::getenv("LIVOX_CTL_PORT")) g_ctl_port = (uint16_t)std::atoi(p);
    g_emit_stdout = (std::getenv("LIVOX_BRIDGE_STDOUT") &&
        std::string(std::getenv("LIVOX_BRIDGE_STDOUT")) == "1");

    // UDP emitter
    g_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_sock < 0) { std::perror("udp socket"); return 3; }
    std::memset(&g_udp_dst, 0, sizeof(g_udp_dst));
    g_udp_dst.sin_family = AF_INET;
    g_udp_dst.sin_addr.s_addr = inet_addr("127.0.0.1");
    g_udp_dst.sin_port = htons(g_emit_port);

    // Init SDK2 (host_ip inferred from JSON; pass "")
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
    if (g_udp_sock >= 0) close(g_udp_sock);
    return 0;
}

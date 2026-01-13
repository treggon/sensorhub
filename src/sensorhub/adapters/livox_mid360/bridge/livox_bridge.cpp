// Livox MID-360 Bridge (SDK2) — Aggregation window + NDJSON/Binary output (no ROS)
// Emits xyzi point frames to UDP localhost. Two modes:
//   - Immediate (default): emit per device packet (small chunks)
//   - Aggregated window: collect points for LIVOX_BRIDGE_AGGREGATE_SEC seconds, then flush
// Output formats:
//   - NDJSON (default): {"type":"points","fields":"xyzi","window":sec,"points":[[x,y,z,i]...]}
//   - Binary (LIVOX_BRIDGE_FORMAT=bin): header 'LVOX' + uint32 N + uint64 ts_us + N*float4(x,y,z,intensity)
//
// Env vars:
//   MID360_CONFIG_PATH        : SDK2 config JSON (contains ip + extrinsic_parameter)
//   LIVOX_UDP_PORT            : UDP port to emit (default 18080)
//   LIVOX_CTL_PORT            : UDP control port (default 18181)
//   LIVOX_BRIDGE_STDOUT       : "1" to also print frames to stdout
//   LIVOX_BRIDGE_AGGREGATE_SEC: e.g., "1.0" to enable aggregated window flushing every N seconds
//   LIVOX_BRIDGE_FORMAT       : "json" (default) or "bin"
//   LIVOX_BRIDGE_FLUSH_CHUNK  : optional max points per NDJSON flush (e.g., 50000). 0 = all

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

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

using namespace std::chrono;

#pragma pack(push, 1)
struct PCLHigh { int32_t x; int32_t y; int32_t z; uint8_t reflectivity; uint8_t tag; };
struct PCLLow  { int16_t x; int16_t y; int16_t z; uint8_t reflectivity; uint8_t tag; };
struct SPHER   { uint32_t depth_mm; uint16_t theta_cdeg; uint16_t phi_cdeg; uint8_t reflectivity; uint8_t tag; };
#pragma pack(pop)
static_assert(sizeof(PCLHigh) == 14, "PCLHigh must be 14 bytes");
static_assert(sizeof(PCLLow)  ==  8, "PCLLow must be 8 bytes");
static_assert(sizeof(SPHER)   == 10, "SPHER must be 10 bytes");

static std::atomic<bool> g_running(true);
static int g_udp_sock = -1;
static sockaddr_in g_udp_dst;
static uint16_t g_emit_port = 18080;
static uint16_t g_ctl_port  = 18181;
static bool g_emit_stdout = false;

static double   g_aggregate_sec = 0.0;         // 0 = immediate; >0 = window aggregation
static bool     g_bin_format     = false;      // false=json NDJSON; true=binary
static uint32_t g_flush_chunk    = 0;          // NDJSON max points per flush (0 = no split)

static std::mutex g_agg_mtx;
static std::vector<std::array<float,4>> g_window_pts; // [x,y,z,intensity]
static uint64_t g_last_flush_us = 0;

static std::vector<uint32_t> g_handles;           // observed handles
static std::mutex g_handles_mtx;

struct Extrinsic {
  double roll_deg{0.0}, pitch_deg{0.0}, yaw_deg{0.0};
  double tx{0.0}, ty{0.0}, tz{0.0};                 // meters
};
static std::map<std::string, Extrinsic> g_extrinsics_by_ip; // ip -> extrinsic
static std::map<uint32_t, std::string> g_handle_ip;         // handle -> ip
static std::mutex g_extr_mtx;

static uint64_t now_us() { return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count(); }
static void emit_ndjson(const std::string& line) {
  if (g_udp_sock >= 0) sendto(g_udp_sock, line.c_str(), (int)line.size(), 0, (struct sockaddr*)&g_udp_dst, sizeof(g_udp_dst));
  if (g_emit_stdout) std::cout << line << std::endl;
}
static void add_handle(uint32_t h) {
  std::lock_guard<std::mutex> lk(g_handles_mtx);
  for (auto v : g_handles) if (v == h) return; g_handles.push_back(h);
}

static std::string find_str(const std::string& s, const std::string& key) {
  size_t p = s.find(key); if (p == std::string::npos) return ""; p = s.find(':', p); if (p == std::string::npos) return "";
  size_t q = s.find('"', p + 1); if (q == std::string::npos) return ""; size_t r = s.find('"', q + 1); if (r == std::string::npos) return "";
  return s.substr(q + 1, r - q - 1);
}
static int find_int(const std::string& s, const std::string& key, int defv) {
  size_t p = s.find(key); if (p == std::string::npos) return defv; p = s.find(':', p); if (p == std::string::npos) return defv;
  char* end = 0; const char* start = s.c_str() + p + 1; long v = std::strtol(start, &end, 10); return (end != start) ? (int)v : defv;
}

static void parse_extrinsics_from_cfg(const char* cfg_path) {
  std::lock_guard<std::mutex> lk(g_extr_mtx);
  g_extrinsics_by_ip.clear(); if (!cfg_path) return; std::ifstream f(cfg_path); if (!f.good()) return;
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>()); size_t pos = 0;
  while (true) {
    size_t ip_key = s.find("\"ip\"", pos); if (ip_key == std::string::npos) break;
    size_t colon = s.find(':', ip_key); size_t q1 = s.find('"', colon + 1); size_t q2 = s.find('"', q1 + 1);
    if (colon == std::string::npos || q1 == std::string::npos || q2 == std::string::npos) break; std::string ip = s.substr(q1 + 1, q2 - q1 - 1);
    size_t ext = s.find("extrinsic_parameter", q2); if (ext == std::string::npos) { pos = q2 + 1; continue; }
    size_t brace = s.find('{', ext); size_t endb = s.find('}', brace); if (brace == std::string::npos || endb == std::string::npos) { pos = q2 + 1; continue; }
    std::string blk = s.substr(brace, endb - brace + 1);
    auto find_num = [&](const char* key, double defv) -> double {
      size_t k = blk.find(key); if (k == std::string::npos) return defv; k = blk.find(':', k); if (k == std::string::npos) return defv;
      char* e = 0; const char* st = blk.c_str() + (k + 1); double v = std::strtod(st, &e); return (e != st) ? v : defv; };
    Extrinsic ex; ex.roll_deg = find_num("roll",0.0); ex.pitch_deg = find_num("pitch",0.0); ex.yaw_deg = find_num("yaw",0.0);
    ex.tx = find_num("x",0.0); ex.ty = find_num("y",0.0); ex.tz = find_num("z",0.0);
    g_extrinsics_by_ip[ip] = ex; pos = endb + 1;
  }
}

struct RotM { double R00,R01,R02,R10,R11,R12,R20,R21,R22; };
static RotM make_rotm(const Extrinsic& ex) {
  const double cr = std::cos(ex.roll_deg*M_PI/180.0), sr = std::sin(ex.roll_deg*M_PI/180.0);
  const double cp = std::cos(ex.pitch_deg*M_PI/180.0), sp = std::sin(ex.pitch_deg*M_PI/180.0);
  const double cy = std::cos(ex.yaw_deg*M_PI/180.0), sy = std::sin(ex.yaw_deg*M_PI/180.0);
  RotM M; M.R00=cy*cp; M.R01=cy*sp*sr - sy*cr; M.R02=cy*sp*cr + sy*sr; M.R10=sy*cp; M.R11=sy*sp*sr + cy*cr; M.R12=sy*sp*cr - cy*sr; M.R20=-sp; M.R21=cp*sr; M.R22=cp*cr; return M;
}

static void ControlAckCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* resp, void*) {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "{\"type\":\"ack\",\"status\":%d,\"handle\":%u,\"ret_code\":%u,\"error_key\":%u}",
    (int)status, handle, resp ? resp->ret_code : 255, resp ? resp->error_key : 0);
  emit_ndjson(buf);
}

static void PointCloudCallback(const uint32_t handle, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* pkt, void*) {
  if (!pkt) return; add_handle(handle);
  const uint8_t* payload = reinterpret_cast<const uint8_t*>(pkt->data);
  const uint32_t n = pkt->dot_num; const uint8_t dt = pkt->data_type; const uint32_t len = pkt->length;

  Extrinsic ex; RotM M; { std::lock_guard<std::mutex> lk(g_extr_mtx); auto itH=g_handle_ip.find(handle); if(itH!=g_handle_ip.end()){ auto itE=g_extrinsics_by_ip.find(itH->second); if(itE!=g_extrinsics_by_ip.end()) ex=itE->second; } M=make_rotm(ex);} 
  auto apply_extrinsic = [&](double& x,double& y,double& z){ double X=M.R00*x+M.R01*y+M.R02*z+ex.tx; double Y=M.R10*x+M.R11*y+M.R12*z+ex.ty; double Z=M.R20*x+M.R21*y+M.R22*z+ex.tz; x=X; y=Y; z=Z; };

  const bool immediate = (g_aggregate_sec <= 0.0);

  if (dt==kLivoxLidarCartesianCoordinateHighData) {
    const size_t stride=sizeof(PCLHigh); const size_t need=(size_t)n*stride; if(len<need) return; auto pts=reinterpret_cast<const PCLHigh*>(payload);
    if (immediate) {
      const uint32_t CHUNK=2000; uint32_t emitted=0; while(emitted<n){ std::string line; line.reserve(CHUNK*32); line+="{\"type\":\"points\",\"fields\":\"xyzi\",\"points\":["; uint32_t take=std::min(CHUNK,n-emitted); for(uint32_t i=0;i<take;++i){ const auto& p=pts[emitted+i]; double x=p.x/1000.0,y=p.y/1000.0,z=p.z/1000.0; apply_extrinsic(x,y,z); char pb[96]; std::snprintf(pb,sizeof(pb),"[%.5f,%.5f,%.5f,%u]%s",x,y,z,(unsigned)p.reflectivity,(i+1<take)?",":""); line+=pb;} line += "]}"; emit_ndjson(line); emitted+=take; }
    } else {
      std::lock_guard<std::mutex> lk(g_agg_mtx);
      for(uint32_t i=0;i<n;++i){ const auto& p=pts[i]; double x=p.x/1000.0,y=p.y/1000.0,z=p.z/1000.0; apply_extrinsic(x,y,z); g_window_pts.emplace_back(std::array<float,4>{(float)x,(float)y,(float)z,(float)p.reflectivity}); }
    }
  } else if (dt==kLivoxLidarCartesianCoordinateLowData) {
    const size_t stride=sizeof(PCLLow); const size_t need=(size_t)n*stride; if(len<need) return; auto pts=reinterpret_cast<const PCLLow*>(payload);
    if (immediate) {
      const uint32_t CHUNK=2000; uint32_t emitted=0; while(emitted<n){ std::string line; line.reserve(CHUNK*32); line+="{\"type\":\"points\",\"fields\":\"xyzi\",\"points\":["; uint32_t take=std::min(CHUNK,n-emitted); for(uint32_t i=0;i<take;++i){ const auto& p=pts[emitted+i]; double x=p.x/100.0,y=p.y/100.0,z=p.z/100.0; apply_extrinsic(x,y,z); char pb[96]; std::snprintf(pb,sizeof(pb),"[%.5f,%.5f,%.5f,%u]%s",x,y,z,(unsigned)p.reflectivity,(i+1<take)?",":""); line+=pb;} line += "]}"; emit_ndjson(line); emitted+=take; }
    } else {
      std::lock_guard<std::mutex> lk(g_agg_mtx);
      for(uint32_t i=0;i<n;++i){ const auto& p=pts[i]; double x=p.x/100.0,y=p.y/100.0,z=p.z/100.0; apply_extrinsic(x,y,z); g_window_pts.emplace_back(std::array<float,4>{(float)x,(float)y,(float)z,(float)p.reflectivity}); }
    }
  } else if (dt==kLivoxLidarSphericalCoordinateData) {
    const size_t stride=sizeof(SPHER); const size_t need=(size_t)n*stride; if(len<need) return; auto pts=reinterpret_cast<const SPHER*>(payload);
    if (immediate) {
      const uint32_t CHUNK=2000; uint32_t emitted=0; while(emitted<n){ std::string line; line.reserve(CHUNK*32); line+="{\"type\":\"points\",\"fields\":\"xyzi\",\"points\":["; uint32_t take=std::min(CHUNK,n-emitted); for(uint32_t i=0;i<take;++i){ const auto& p=pts[emitted+i]; const double r=p.depth_mm/1000.0; const double th=(p.theta_cdeg/100.0)*(M_PI/180.0); const double ph=(p.phi_cdeg/100.0)*(M_PI/180.0); double x=r*std::cos(ph)*std::cos(th), y=r*std::cos(ph)*std::sin(th), z=r*std::sin(ph); apply_extrinsic(x,y,z); char pb[96]; std::snprintf(pb,sizeof(pb),"[%.5f,%.5f,%.5f,%u]%s",x,y,z,(unsigned)p.reflectivity,(i+1<take)?",":""); line+=pb;} line += "]}"; emit_ndjson(line); emitted+=take; }
    } else {
      std::lock_guard<std::mutex> lk(g_agg_mtx);
      for(uint32_t i=0;i<n;++i){ const auto& p=pts[i]; const double r=p.depth_mm/1000.0; const double th=(p.theta_cdeg/100.0)*(M_PI/180.0); const double ph=(p.phi_cdeg/100.0)*(M_PI/180.0); double x=r*std::cos(ph)*std::cos(th), y=r*std::cos(ph)*std::sin(th), z=r*std::sin(ph); apply_extrinsic(x,y,z); g_window_pts.emplace_back(std::array<float,4>{(float)x,(float)y,(float)z,(float)p.reflectivity}); }
    }
  } else { return; }
}

static void maybe_flush_window() {
  if (g_aggregate_sec <= 0.0) return;
  const uint64_t now = now_us();
  const double   elapsed = (now - g_last_flush_us) / 1e6;
  if (elapsed < g_aggregate_sec) return;

  std::vector<std::array<float,4>> to_emit;
  {
    std::lock_guard<std::mutex> lk(g_agg_mtx);
    if (g_window_pts.empty()) { g_last_flush_us = now; return; }
    to_emit.swap(g_window_pts);
  }

  if (!g_bin_format) {
    const uint32_t CHUNK = (g_flush_chunk ? g_flush_chunk : (uint32_t)to_emit.size());
    uint32_t start = 0;
    while (start < to_emit.size()) {
      const uint32_t take = std::min(CHUNK, (uint32_t)(to_emit.size() - start));
      std::string line; line.reserve(take * 32);
      line += "{\"type\":\"points\",\"fields\":\"xyzi\",\"window\":";
      line += std::to_string(g_aggregate_sec);
      line += ",\"points\":[";
      for (uint32_t i=0;i<take;i++) {
        const auto& p = to_emit[start+i];
        char buf[96];
        std::snprintf(buf,sizeof(buf),"[%.5f,%.5f,%.5f,%u]%s", p[0],p[1],p[2], (unsigned)std::lround(p[3]), (i+1<take)?",":"");
        line += buf;
      }
      line += "]}";
      emit_ndjson(line);
      start += take;
    }
  } else {
    const uint32_t N   = (uint32_t)to_emit.size();
    const size_t   H   = 4 + 4 + 8;
    const size_t   B   = N * sizeof(float) * 4;
    std::string    pkt; pkt.resize(H + B);
    pkt[0]='L'; pkt[1]='V'; pkt[2]='O'; pkt[3]='X';
    *reinterpret_cast<uint32_t*>(&pkt[4])  = N;
    *reinterpret_cast<uint64_t*>(&pkt[8])  = now;
    float* f = reinterpret_cast<float*>(&pkt[H]);
    for (uint32_t i=0;i<N;i++) { const auto& p = to_emit[i]; f[i*4+0]=p[0]; f[i*4+1]=p[1]; f[i*4+2]=p[2]; f[i*4+3]=p[3]; }
    if (g_udp_sock >= 0) sendto(g_udp_sock, pkt.data(), (int)pkt.size(), 0, (struct sockaddr*)&g_udp_dst, sizeof(g_udp_dst));
    if (g_emit_stdout) std::fwrite(pkt.data(), 1, pkt.size(), stdout);
  }

  g_last_flush_us = now;
}

static void ImuCallback(const uint32_t handle, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* pkt, void*) {
  if (!pkt) return; add_handle(handle);
  if (pkt->length < sizeof(LivoxLidarImuRawPoint)) return;
  const auto* imu = reinterpret_cast<const LivoxLidarImuRawPoint*>(pkt->data);
  unsigned long long ts_us = (unsigned long long)now_us();
  char buf[256];
  std::snprintf(buf, sizeof(buf),
    "{\"type\":\"imu\",\"ts_us\":%llu,\"handle\":%u,\"ax\":%.6f,\"ay\":%.6f,\"az\":%.6f,\"gx\":%.6f,\"gy\":%.6f,\"gz\":%.6f}",
    ts_us, handle, imu->acc_x, imu->acc_y, imu->acc_z, imu->gyro_x, imu->gyro_y, imu->gyro_z);
  emit_ndjson(buf);
}

static void InfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void*) {
  add_handle(handle); if(!info) return;
  { std::lock_guard<std::mutex> lk(g_extr_mtx); g_handle_ip[handle] = std::string(info->lidar_ip); }
  char buf[256];
  std::snprintf(buf,sizeof(buf),
    "{\"type\":\"info\",\"handle\":%u,\"dev_type\":%u,\"sn\":\"%.16s\",\"ip\":\"%.16s\"}",
    handle, info->dev_type, info->sn, info->lidar_ip);
  emit_ndjson(buf);
}

static void handle_command(const std::string& msg) {
  const std::string cmd = find_str(msg, "cmd");
  if (cmd=="set_work_mode") { const LivoxLidarWorkMode wm=(LivoxLidarWorkMode)find_int(msg,"mode",(int)kLivoxLidarNormal); std::lock_guard<std::mutex> lk(g_handles_mtx); for(auto h: g_handles) SetLivoxLidarWorkMode(h, wm, ControlAckCallback, NULL); }
  else if (cmd=="set_pattern_mode") { const LivoxLidarScanPattern sp=(LivoxLidarScanPattern)find_int(msg,"pattern_mode",(int)kLivoxLidarScanPatternNoneRepetive); std::lock_guard<std::mutex> lk(g_handles_mtx); for(auto h: g_handles) SetLivoxLidarScanPattern(h, sp, ControlAckCallback, NULL); }
  else if (cmd=="set_fov") { FovCfg cfg; cfg.yaw_start=find_int(msg,"yaw_start",0); cfg.yaw_stop=find_int(msg,"yaw_stop",0); cfg.pitch_start=find_int(msg,"pitch_start",-7); cfg.pitch_stop=find_int(msg,"pitch_stop",52); cfg.rsvd=0; const int en=find_int(msg,"enable",1); std::lock_guard<std::mutex> lk(g_handles_mtx); for(auto h: g_handles){ SetLivoxLidarFovCfg1(h,&cfg,ControlAckCallback,NULL); EnableLivoxLidarFov(h,(uint8_t)en,ControlAckCallback,NULL);} }
  else if (cmd=="set_imu_enable") { const int en=find_int(msg,"enable",1); std::lock_guard<std::mutex> lk(g_handles_mtx); for(auto h: g_handles){ if(en) EnableLivoxLidarImuData(h,ControlAckCallback,NULL); else DisableLivoxLidarImuData(h,ControlAckCallback,NULL);} }
  else if (cmd=="set_time_sync") { const std::string rmc=find_str(msg,"rmc"); if(!rmc.empty()){ std::lock_guard<std::mutex> lk(g_handles_mtx); for(auto h: g_handles) SetLivoxLidarRmcSyncTime(h, rmc.c_str(), (uint16_t)rmc.size(), NULL, NULL);} }
  else if (cmd=="set_extrinsic") { const std::string ip=find_str(msg,"ip"); double roll=(double)find_int(msg,"roll",0), pitch=(double)find_int(msg,"pitch",0), yaw=(double)find_int(msg,"yaw",0); auto fd=[&](const std::string& k,double d){ size_t p=msg.find(k); if(p==std::string::npos) return d; p=msg.find(':',p); if(p==std::string::npos) return d; char* e=0; const char* st=msg.c_str()+p+1; double v=strtod(st,&e); return (e!=st)?v:d; }; Extrinsic ex; ex.roll_deg=roll; ex.pitch_deg=pitch; ex.yaw_deg=yaw; ex.tx=fd("tx",0.0); ex.ty=fd("ty",0.0); ex.tz=fd("tz",0.0); { std::lock_guard<std::mutex> lk(g_extr_mtx); if(!ip.empty()) g_extrinsics_by_ip[ip]=ex; else for(auto& kv: g_extrinsics_by_ip) kv.second=ex; } emit_ndjson("{\"type\":\"ack\",\"cmd\":\"set_extrinsic\",\"status\":0}"); }
}

static void control_thread() {
  int sock=socket(AF_INET,SOCK_DGRAM,0); if(sock<0){ std::perror("control socket"); return; }
  sockaddr_in addr; std::memset(&addr,0,sizeof(addr)); addr.sin_family=AF_INET; addr.sin_addr.s_addr=inet_addr("127.0.0.1"); addr.sin_port=htons(g_ctl_port);
  if(bind(sock,(struct sockaddr*)&addr,sizeof(addr))<0){ std::perror("control bind"); close(sock); return; }
  char buf[4096]; while(g_running.load()){ sockaddr_in src; socklen_t sl=sizeof(src); int n=recvfrom(sock,buf,sizeof(buf)-1,0,(struct sockaddr*)&src,&sl); if(n>0){ buf[n]='\0'; handle_command(std::string(buf)); } else { std::this_thread::sleep_for(std::chrono::milliseconds(5)); } } close(sock);
}

static void on_sigint(int){ g_running.store(false); }

int main(int, char**) {
  signal(SIGINT,on_sigint);
  const char* cfg_path = std::getenv("MID360_CONFIG_PATH"); if(!cfg_path||std::strlen(cfg_path)==0){ std::cerr << "MID360_CONFIG_PATH env var is required (SDK2 JSON)." << std::endl; return 2; }
  if (const char* p = std::getenv("LIVOX_UDP_PORT")) g_emit_port = (uint16_t)std::atoi(p);
  if (const char* p = std::getenv("LIVOX_CTL_PORT")) g_ctl_port  = (uint16_t)std::atoi(p);
  g_emit_stdout = (std::getenv("LIVOX_BRIDGE_STDOUT") && std::string(std::getenv("LIVOX_BRIDGE_STDOUT")) == "1");
  if (const char* s = std::getenv("LIVOX_BRIDGE_AGGREGATE_SEC")) g_aggregate_sec = std::atof(s);
  if (const char* s = std::getenv("LIVOX_BRIDGE_FORMAT")) g_bin_format = (std::string(s) == "bin");
  if (const char* s = std::getenv("LIVOX_BRIDGE_FLUSH_CHUNK")) g_flush_chunk = (uint32_t)std::atoi(s);

  g_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_udp_sock < 0) { std::perror("udp socket"); return 3; }
  std::memset(&g_udp_dst, 0, sizeof(g_udp_dst)); g_udp_dst.sin_family = AF_INET; g_udp_dst.sin_addr.s_addr = inet_addr("127.0.0.1"); g_udp_dst.sin_port = htons(g_emit_port);
  parse_extrinsics_from_cfg(cfg_path);
  g_last_flush_us = now_us();

  if (!LivoxLidarSdkInit(cfg_path, "", NULL)) { std::cerr << "LivoxLidarSdkInit failed." << std::endl; return 4; }
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, NULL);
  SetLivoxLidarImuDataCallback(ImuCallback, NULL);
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, NULL);
  if (!LivoxLidarSdkStart()) { std::cerr << "LivoxLidarSdkStart failed." << std::endl; LivoxLidarSdkUninit(); return 5; }

  std::thread ctl(control_thread);
  while (g_running.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); maybe_flush_window(); }
  LivoxLidarSdkUninit();
  if (ctl.joinable()) ctl.join();
  if (g_udp_sock >= 0) close(g_udp_sock);
  return 0;
}

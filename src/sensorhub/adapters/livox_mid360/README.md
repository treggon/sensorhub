
# Livox Mid‑360 Integration (SensorHub)

This bundle adds a **multi‑device Livox Mid‑360 adapter** and a **C++ bridge** that reads
**Livox SDK2** callbacks and publishes **NDJSON** frames to SensorHub via UDP/STDOUT.
It includes **JSON config**, **schema validation**, and **step‑by‑step build instructions**.

## Repo paths
```
src/sensorhub/adapters/livox_mid360/livox_adapter.py
src/sensorhub/adapters/livox_mid360/bridge/CMakeLists.txt
src/sensorhub/adapters/livox_mid360/bridge/livox_bridge.cpp
src/sensorhub/config/mid360_config.json
src/sensorhub/config/mid360_schema.json
```

## 1) Livox SDK2 install (Ubuntu/ARM)
```bash
sudo apt update
sudo apt install -y git cmake build-essential
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
# Header fix for Ubuntu 24.04/GCC 13
sed -i '1i #include <cstdint>' sdk_core/comm/define.h
rm -rf build && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_CXX_STANDARD_REQUIRED=ON \
      -DCMAKE_CXX_EXTENSIONS=OFF \
      -DCMAKE_CXX_FLAGS="-Wno-c++20-compat" \
      .. && make -j"$(nproc)"
sudo make install
```
- SDK2 provides sample callbacks & types; we follow those in the bridge.  
- Mid‑360 uses UDP for control/data; defaults are 56000/57000/58000.  
**Refs:** SDK2 README & sample, Mid‑360 protocol. citeturn1search25turn1search19turn1search29

## 2) Configure multiple Mid‑360s (JSON)
Edit `src/sensorhub/config/mid360_config.json`:
```json
{
  "lidars": [
    {"id":"mid360_front","lidar_ip":"10.0.0.10","host_ip":"10.0.0.20",
     "cmd_data_port":56000,"point_data_port":57000,"imu_data_port":58000},
    {"id":"mid360_rear","lidar_ip":"10.0.0.11","host_ip":"10.0.0.20",
     "cmd_data_port":56000,"point_data_port":57000,"imu_data_port":58000,
     "ndjson_udp_port":18081}
  ],
  "bridge": {"ndjson_udp_port":18080, "stdout": false}
}
```
Validation runs in the adapter against `mid360_schema.json` (IPv4, port ranges, required keys).

## 3) Build the C++ bridge
```bash
cd src/sensorhub/adapters/livox_mid360/bridge
mkdir build && cd build
cmake .. && make -j"$(nproc)"
```
If `nlohmann_json` is found, the bridge parses JSON directly; otherwise it falls back to env variables.
The bridge tags every frame with `lidar_id` and emits NDJSON to one global UDP port or per‑device ports.

## 4) Run SensorHub
Add the adapter router to your app (if not already):
```python
from sensorhub.adapters.livox_mid360.livox_adapter import router as livox_router
app.include_router(livox_router)
```
Start SensorHub:
```bash
uvicorn sensorhub.main:app --host 0.0.0.0 --port 8080
```
The adapter loads & **validates** the JSON config, then starts the bridge. It listens on `127.0.0.1:18080` by default.

## 5) Use the API
- `GET /lidar/info`    → status + buffer sizes
- `GET /lidar/config`  → the validated JSON config
- `GET /lidar/points/latest`  → latest frame (contains `lidar_id`)
- `GET /lidar/points/recent?count=N` → last N frames
- `WS /lidar/ws`       → live NDJSON frames (filter by `lidar_id` client-side)

## Notes
- The callback signatures and data structs are identical to the official SDK2 sample (`LivoxLidarEthernetPacket`,
  Cartesian/Spherical point types). We emit a small sample of points per frame for efficiency, matching the sample’s approach. **Ref:** SDK2 sample. citeturn1search19
- Mid‑360 protocol and default ports documented by Livox; UDP transport is used for data/control. **Ref:** Mid‑360 protocol. citeturn1search29
- The bridge resolves device identity (`lidar_id`) from the SDK **handle** via IP mapping from your JSON. This mirrors the Livox ROS driver’s handle‑centric integration notes. **Ref:** ROS driver integration. citeturn1search21


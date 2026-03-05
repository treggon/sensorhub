
# src/sensorhub/adapters/livox_mid360/livox_adapter.py (no ROS)
"""
Livox MID-360 adapter — listens to livox_bridge NDJSON UDP and publishes normalized point cloud + IMU.
- No ROS dependencies. Systemd control optional.
- Points are expected as floats (meters) with intensity from the bridge.

References:
- MID-360 protocol: data types and units (mm/cm/centideg) (protocol)  [1](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)
- Livox ROS2 driver processing notes (structure fields, normalization) (notes)  [4](https://deepwiki.com/Livox-SDK/livox_ros_driver2/4.1-point-cloud-processing)
- MID-360 User Manual (Viewer 2 features overview) (manual)  [2](https://www.sachtleben-technology.com/wp-content/uploads/2024/07/LivoxMid-360UserManual.pdf)
"""

import os
import time
import socket
import struct
import select
import subprocess
import threading
import logging
import math
import json
from typing import Optional, List, Tuple, Dict, Any

class LivoxMid360Adapter:
    DATA_TYPE_CARTESIAN_HIGH = "cartesian_high"
    DATA_TYPE_CARTESIAN_LOW  = "cartesian_low"
    DATA_TYPE_SPHERICAL      = "spherical"

    def __init__(
        self,
        sensor_id: str,
        config_path: str,
        use_systemd: bool = True,
        service_name: str = "livox_bridge",
        bridge_path: Optional[str] = None,
        bridge_udp_port: Optional[int] = None,
        publish_period: float = 0.5,
        frame_ms: int = 100,
        max_points: int = 20000,
        keep_fields: str = "xyzi",
        decimals: int = 3,
    ) -> None:
        self.sensor_id = sensor_id
        self.logger = logging.getLogger(f"livox.adapter.{sensor_id}")
        self._stop = threading.Event()
        self.config_path = os.path.abspath(config_path)
        self.use_systemd = use_systemd
        self.service_name = service_name
        self.bridge_path = bridge_path
        self.bridge_udp_port = int(bridge_udp_port or os.getenv("LIVOX_UDP_PORT", "18080"))
        self.publish_period = float(publish_period)
        self.frame_ms = int(frame_ms)
        self.max_points = int(max_points)
        self.keep_fields = str(keep_fields).lower()
        self.decimals = int(decimals)
        self._bridge_sock: Optional[socket.socket] = None
        self._proc: Optional[subprocess.Popen] = None
        self._frame_buf: List[Tuple[float,float,float,int]] = []
        self._frame_start = time.time()
        self._point_pkts = 0
        self._point_bytes = 0
        self._imu_pkts = 0
        self._imu_bytes = 0
        self._last_point_ts = 0.0
        self._last_imu_ts = 0.0
        self._imu_last: Optional[Dict[str,float]] = None
        self._last_data_type: Optional[str] = None
        self._last_stride: Optional[int] = None
        self._last_payload_len: int = 0
        self._decoded_pts_last: int = 0
        self._last_header: Optional[Dict[str,int]] = None
        self._ctl_port = int(os.getenv("LIVOX_CTL_PORT", "18181"))

    # ---------------------- systemd/bridge management ----------------------
    def _unit_name(self, name: str) -> str:
        return name if name.endswith('.service') else f"{name}.service"

    def _run_systemctl_user(self, args: List[str], **kwargs) -> subprocess.CompletedProcess:
        cmd = ["/usr/bin/systemctl", "--user"] + args
        return subprocess.run(cmd, **kwargs)

    def start(self) -> None:
        if not os.path.isfile(self.config_path):
            raise RuntimeError(f"Livox config JSON not found: {self.config_path}")
        if self.use_systemd:
            try:
                unit = self._unit_name(self.service_name)
                chk = self._run_systemctl_user(["is-active", unit], capture_output=True, text=True)
                if chk.stdout.strip() != "active":
                    self._run_systemctl_user(["start", unit], check=True)
                self.logger.info("Started user systemd unit '%s'", unit)
            except Exception as e:
                raise RuntimeError(f"systemd start failed for {self.service_name}: {e}")
        else:
            if not self.bridge_path or not os.path.isfile(self.bridge_path):
                raise RuntimeError("bridge_path is required when use_systemd=False")
            self._proc = subprocess.Popen(
                [self.bridge_path, self.config_path],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=1, universal_newlines=True,
            )
        # Bind NDJSON UDP
        self._bridge_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self._bridge_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._bridge_sock.bind(("127.0.0.1", self.bridge_udp_port))
        self._bridge_sock.setblocking(False)
        self.logger.info("Listening for bridge NDJSON on 127.0.0.1:%d", self.bridge_udp_port)

    def stop(self) -> None:
        if self._bridge_sock:
            try: self._bridge_sock.close()
            except Exception: pass
            self._bridge_sock = None
        if self.use_systemd:
            try:
                unit = self._unit_name(self.service_name)
                self._run_systemctl_user(["stop", unit], check=True)
            except Exception as e:
                self.logger.warning("systemd stop failed: %s", e)
        else:
            if self._proc:
                try:
                    self._proc.terminate(); self._proc.wait(timeout=3.0)
                except Exception:
                    try: self._proc.kill()
                    except Exception: pass
                finally: self._proc = None

    # ---------------------- UDP parsing ----------------------
    def _parse_bridge_ndjson(self, raw: bytes) -> None:
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                return
            obj = json.loads(line)
            t = obj.get("type")
            if t == "points":
                pts = obj.get("points") or []
                for p in pts:
                    if isinstance(p, list) and len(p) >= 4:
                        x,y,z,i = float(p[0]), float(p[1]), float(p[2]), int(p[3])
                        # Basic sanity filtering
                        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and \
                           max(abs(x),abs(y)) < 200.0 and abs(z) < 200.0:
                            self._frame_buf.append((x,y,z,i))
                self._last_data_type = obj.get("data_type")
                self._last_stride = 16
                self._last_payload_len = len(raw)
                self._decoded_pts_last = len(pts)
                self._last_header = {"seq": obj.get("seq"), "extrinsic_applied": obj.get("extrinsic_applied")}
            elif t == "imu":
                self._imu_last = {
                    "gx": float(obj.get("gx_radps", 0.0)),
                    "gy": float(obj.get("gy_radps", 0.0)),
                    "gz": float(obj.get("gz_radps", 0.0)),
                    "ax": float(obj.get("ax_g", 0.0)),
                    "ay": float(obj.get("ay_g", 0.0)),
                    "az": float(obj.get("az_g", 0.0)),
                }
            else:
                pass
        except Exception as e:
            self.logger.debug("Bridge NDJSON parse error: %s", e)

    def run(self) -> None:
        self.logger.info("Livox MID-360 adapter run() loop started.")
        last_pub = time.time()
        self._frame_start = last_pub
        try:
            while not self._stop.is_set():
                rlist = [s for s in (self._bridge_sock,) if s]
                rs, _, _ = select.select(rlist, [], [], 0.01)
                for s in rs:
                    try:
                        data, _addr = s.recvfrom(65535)
                        now = time.time()
                        self._point_pkts += 1
                        self._point_bytes += len(data)
                        self._last_point_ts = now
                        self._parse_bridge_ndjson(data)
                    except Exception:
                        pass
                now = time.time()
                should_pub = (now - last_pub) >= self.publish_period
                frame_due = (now - self._frame_start) >= (self.frame_ms / 1000.0)
                if should_pub or frame_due:
                    payload: Dict[str, Any] = {
                        "sensor_id": self.sensor_id,
                        "status": "running",
                        "point_pkts": self._point_pkts,
                        "point_bytes": self._point_bytes,
                        "imu_pkts": self._imu_pkts,
                        "imu_bytes": self._imu_bytes,
                        "last_point_ts": self._last_point_ts,
                        "last_imu_ts": self._last_imu_ts,
                        "timestamp": now,
                        "decode_diag": {
                            "data_type": self._last_data_type,
                            "stride": self._last_stride,
                            "payload_len": self._last_payload_len,
                            "decoded_pts_last": self._decoded_pts_last,
                            "header": self._last_header,
                        }
                    }
                    if self._frame_buf:
                        pts = self._frame_buf
                        if len(pts) > self.max_points:
                            step = max(1, len(pts) // self.max_points)
                            pts = pts[::step]
                        payload["points"] = self._compact_points(pts)
                        payload["frame_points"] = len(pts)
                        payload["points_format"] = {"data_type": self._last_data_type, "stride": self._last_stride}
                    if self._imu_last:
                        payload["imu"] = {
                            "gx_radps": self._imu_last["gx"],
                            "gy_radps": self._imu_last["gy"],
                            "gz_radps": self._imu_last["gz"],
                            "ax_g": self._imu_last["ax"],
                            "ay_g": self._imu_last["ay"],
                            "az_g": self._imu_last["az"],
                            "ts": self._last_imu_ts,
                        }
                    # Replace with your publish mechanism
                    self._publish(payload)
                    last_pub = now
                    if frame_due:
                        self._frame_buf = []
                        self._frame_start = now
                time.sleep(0.002)
        except Exception as e:
            self.logger.error("Livox MID-360 run-loop error: %s", e)
        finally:
            self.logger.info("Livox MID-360 run() loop exiting.")

    def _compact_points(self, pts: List[Tuple[float,float,float,int]]) -> List[Tuple]:
        keep = {
            'xy': (0,1),
            'xyz': (0,1,2),
            'xyi': (0,1,3),
            'xyzi': (0,1,2,3),
        }.get(self.keep_fields, (0,1,2,3))
        dec = max(0, self.decimals)
        out: List[Tuple] = []
        for (x,y,z,i) in pts:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            vals = (x,y,z,i)
            row: List[Any] = []
            for idx in keep:
                if idx in (0,1,2):
                    row.append(round(vals[idx], dec))
                else:
                    row.append(int(vals[idx]))
            out.append(tuple(row))
        return out

    # --------------- Publish stub ---------------
    def _publish(self, payload: Dict[str, Any]) -> None:
        # Replace with your bus/logger; for now print brief stats for sanity
        print(json.dumps({
            "sensor_id": payload.get("sensor_id"),
            "frame_points": payload.get("frame_points", 0),
            "data_type": payload.get("points_format", {}).get("data_type"),
        }))

    # --------------- Bridge control convenience ---------------
    def send_control(self, obj: Dict[str, Any]) -> None:
        try:
            msg = json.dumps(obj).encode("utf-8")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
            sock.sendto(msg, ("127.0.0.1", self._ctl_port))
            sock.close()
        except Exception as e:
            self.logger.warning("Bridge control send failed: %s", e)

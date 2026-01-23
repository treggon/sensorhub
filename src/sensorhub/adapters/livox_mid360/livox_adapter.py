
# src/sensorhub/adapters/livox_mid360/livox_adapter.py
"""
Livox MID-360 UDP listener + point/IMU decoder (optimized).
- Controls the bridge via systemd user unit or spawns it directly.
- Listens to multicast UDP for point cloud + IMU.
- Aggregates a short frame window and publishes a compact payload.
- Exposes FastAPI routes under /livox (service controls + IMU toggle).

Health model (adapter-level):
- ok      : fresh data in < ok_stale_sec
- warning : service active + running, but no data yet or data stale < err_stale_sec
- error   : adapter not running OR service not active/failed OR data stale >= err_stale_sec
"""
from __future__ import annotations
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
from fastapi import APIRouter, HTTPException, Response
from sensorhub.core.sensor_base import AbstractSensorAdapter

# --- FastAPI router (exported) ---
router = APIRouter(prefix="/livox", tags=["livox"])  # <-- imported by main.py
__all__ = ["router", "LivoxMid360Adapter"]

# --- helpers ---
def _unit_name(name: str) -> str:
    return name if name.endswith(".service") else f"{name}.service"

def _run_systemctl_user(args: List[str], **kwargs) -> subprocess.CompletedProcess:
    cmd = ["/usr/bin/systemctl", "--user"] + args
    return subprocess.run(cmd, **kwargs)

@router.get("/service/status")
def livox_service_status(service_name: str = "livox_bridge"):
    try:
        unit = _unit_name(service_name)
        result = _run_systemctl_user(["is-active", unit], capture_output=True, text=True)
        return {"service": unit, "status": result.stdout.strip(), "code": result.returncode}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"systemd status failed: {e}")

@router.post("/service/start")
def livox_service_start(service_name: str = "livox_bridge"):
    try:
        unit = _unit_name(service_name)
        chk = _run_systemctl_user(["is-active", unit], capture_output=True, text=True)
        if chk.stdout.strip() != "active":
            _run_systemctl_user(["start", unit], check=True)
        return {"service": unit, "action": "start", "ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"systemd start failed: {e}")

@router.post("/service/stop")
def livox_service_stop(service_name: str = "livox_bridge"):
    try:
        unit = _unit_name(service_name)
        _run_systemctl_user(["stop", unit], check=True)
        return {"service": unit, "action": "stop", "ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"systemd stop failed: {e}")

# --- adapter ---
class LivoxMid360Adapter(AbstractSensorAdapter):
    DATA_TYPE_CARTESIAN_HIGH = 0x00
    DATA_TYPE_CARTESIAN_LOW  = 0x01
    DATA_TYPE_SPHERICAL      = 0x02

    def __init__(
        self,
        sensor_id: str,
        kind: str = "lidar3d",
        config_path: str = "/home/dev/treggon/sensorhub/src/sensorhub/config/mid360_config.json",
        use_systemd: bool = True,
        service_name: str = "livox_bridge",
        bridge_path: Optional[str] = None,
        multicast_ip: str = "224.1.1.5",
        point_port: int = 56301,
        imu_port: int = 56401,
        listen_udp: bool = True,
        publish_period: float = 0.5,
        hz: Optional[float] = None,
        decode_points: bool = True,
        frame_ms: int = 100,
        max_points: int = 20000,
        keep_fields: str = "xyzi",
        decimals: int = 2,
        # --- NEW health knobs ---
        ok_stale_sec: float = 1.0,
        warn_stale_sec: float = 3.0,
        err_stale_sec: float = 10.0,
        **kwargs,
    ) -> None:
        super().__init__(sensor_id, kind)
        self.logger = getattr(self, "logger", logging.getLogger(f"sensorhub.adapters.livox_mid360.{self.__class__.__name__}.{sensor_id}"))
        self._stop = getattr(self, "_stop", threading.Event())
        self.config_path = os.path.abspath(config_path)
        self.use_systemd = use_systemd
        self.service_name = service_name
        self.bridge_path = bridge_path
        self.multicast_ip = multicast_ip
        self.point_port = int(point_port)
        self.imu_port = int(imu_port)
        self.listen_udp = bool(listen_udp)
        self.publish_period = (1.0 / hz) if (hz and hz > 0) else float(publish_period)

        self._pt_sock: Optional[socket.socket] = None
        self._imu_sock: Optional[socket.socket] = None
        self._proc: Optional[subprocess.Popen] = None

        self._point_pkts = 0
        self._point_bytes = 0
        self._imu_pkts = 0
        self._imu_bytes = 0
        self._last_point_ts = 0.0
        self._last_imu_ts = 0.0

        self.decode_points = bool(decode_points)
        self.frame_ms = int(frame_ms)
        self.max_points = int(max_points)
        self.keep_fields = str(keep_fields).lower()
        self.decimals = int(decimals)
        self._frame_buf: List[Tuple[float, float, float, int]] = []
        self._frame_start = time.time()

        # diagnostics
        self._last_data_type: Optional[int] = None
        self._last_stride: Optional[int] = None
        self._last_payload_len: int = 0
        self._last_header: Optional[Dict[str, int]] = None
        self._last_pkt_raw: Optional[bytes] = None
        self._decoded_pts_last: int = 0

        # latest IMU sample (gyro rad/s; accel g)
        self._imu_last: Optional[Dict[str, float]] = None

        # Bridge control port (UDP localhost)
        self._ctl_port = int(os.getenv("LIVOX_CTL_PORT", "18181"))

        # --- NEW: thresholds for health classification ---
        self.ok_stale_sec = float(ok_stale_sec)
        self.warn_stale_sec = float(warn_stale_sec)
        self.err_stale_sec = float(err_stale_sec)

    # ---- systemd / bridge management ----
    def _systemd_start(self) -> None:
        try:
            unit = _unit_name(self.service_name)
            chk = _run_systemctl_user(["is-active", unit], capture_output=True, text=True)
            if chk.stdout.strip() != "active":
                _run_systemctl_user(["start", unit], check=True)
            self.logger.info("Started user systemd unit '%s'", unit)
        except Exception as e:
            raise RuntimeError(f"systemd start failed for {self.service_name}: {e}")

    def _systemd_stop(self) -> None:
        try:
            unit = _unit_name(self.service_name)
            _run_systemctl_user(["stop", unit], check=True)
            self.logger.info("Stopped user systemd unit '%s'", unit)
        except Exception as e:
            self.logger.warning("systemd stop failed for %s: %s", self.service_name, e)

    def _spawn_bridge(self) -> None:
        if not self.bridge_path:
            raise RuntimeError("bridge_path is required when use_systemd=False")
        if not os.path.isfile(self.bridge_path):
            raise RuntimeError(f"livox_bridge not found: {self.bridge_path}")
        if not os.path.isfile(self.config_path):
            raise RuntimeError(f"Livox config JSON not found: {self.config_path}")
        self._proc = subprocess.Popen(
            [self.bridge_path, self.config_path],
            cwd=os.path.dirname(self.bridge_path) or None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            universal_newlines=True,
        )
        self.logger.info("Spawned livox_bridge: %s %s", self.bridge_path, self.config_path)

    def _terminate_bridge(self) -> None:
        if self._proc:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3.0)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            finally:
                self._proc = None

    # ---- UDP sockets ----
    def _join_multicast(self, port: int) -> socket.socket:
        iface = os.getenv('LIVOX_IFACE', '0.0.0.0')
        rcvbuf = int(os.getenv('LIVOX_RCVBUF', '4194304'))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        except Exception:
            pass
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((iface, port))
        except Exception as e:
            raise RuntimeError(f"Failed to bind UDP port {port}: {e}")
        mreq = struct.pack("4s4s", socket.inet_aton(self.multicast_ip), socket.inet_aton("0.0.0.0"))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.setblocking(False)
        self.logger.info("Joined multicast %s on UDP port %d (iface=%s, rcvbuf=%d)", self.multicast_ip, port, iface, rcvbuf)
        return sock

    # ---- start/stop ----
    def start(self) -> None:
        if not os.path.isfile(self.config_path):
            raise RuntimeError(f"Livox config JSON not found: {self.config_path}")
        self.logger.info("Using Livox config: %s", self.config_path)
        if self.use_systemd:
            self._systemd_start()
        else:
            self._spawn_bridge()
        if self.listen_udp:
            try:
                self._pt_sock = self._join_multicast(self.point_port)
            except Exception as e:
                self.logger.warning("Point cloud listener setup failed: %s", e)
                self._pt_sock = None
            try:
                self._imu_sock = self._join_multicast(self.imu_port)
            except Exception as e:
                self.logger.warning("IMU listener setup failed: %s", e)
                self._imu_sock = None
        super().start()

    def stop(self) -> None:
        for s in (self._pt_sock, self._imu_sock):
            if s:
                try:
                    s.close()
                except Exception:
                    pass
        self._pt_sock = None
        self._imu_sock = None
        if self.use_systemd:
            self._systemd_stop()
        else:
            self._terminate_bridge()
        super().stop()

    # ---- point decoding ----
    def _decode_cartesian_high(self, payload: bytes) -> int:
        npts = 0
        if len(payload) % 14 != 0:
            return 0
        n = len(payload) // 14
        off = 0
        for _ in range(n):
            x_mm, y_mm, z_mm, refl, tag = struct.unpack_from("<iiiBB", payload, off)
            off += 14
            self._frame_buf.append((x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, int(refl)))
            npts += 1
        return npts

    def _decode_cartesian_high_autoskip(self, payload: bytes) -> Tuple[int, int]:
        stride = 14
        for skip in range(0, 65, 2):
            rem = payload[skip:]
            if len(rem) <= 0 or (len(rem) % stride) != 0:
                continue
            off = 0
            ok = 0
            sample = min(20, len(rem) // stride)
            for _ in range(sample):
                x_mm, y_mm, z_mm, refl, tag = struct.unpack_from("<iiiBB", rem, off)
                off += stride
                x = x_mm / 1000.0; y = y_mm / 1000.0; z = z_mm / 1000.0
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and max(abs(x), abs(y)) < 50.0 and abs(z) < 10.0:
                    ok += 1
            if ok >= max(5, sample // 2):
                off = 0
                pts_local = []
                for _ in range(len(rem) // stride):
                    x_mm, y_mm, z_mm, refl, tag = struct.unpack_from("<iiiBB", rem, off)
                    off += stride
                    pts_local.append((x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, int(refl)))
                self._frame_buf.extend(pts_local)
                return (len(pts_local), skip)
        return (0, 0)

    def _decode_cartesian_low(self, payload: bytes) -> int:
        npts = 0
        if len(payload) % 8 != 0:
            return 0
        n = len(payload) // 8
        off = 0
        for _ in range(n):
            x_cm, y_cm, z_cm, refl, tag = struct.unpack_from("<hhhBB", payload, off)
            off += 8
            self._frame_buf.append((x_cm / 100.0, y_cm / 100.0, z_cm / 100.0, int(refl)))
            npts += 1
        return npts

    def _decode_spherical(self, payload: bytes) -> int:
        npts = 0
        stride = 10
        if len(payload) % stride != 0:
            return 0
        n = len(payload) // stride
        off = 0
        for _ in range(n):
            # depth uint32 (mm), theta int16 (centideg), phi int16 (centideg)
            depth_mm, theta_cdeg, phi_cdeg, refl, tag = struct.unpack_from("<IhhBB", payload, off)
            off += stride
            r = depth_mm / 1000.0
            theta = (theta_cdeg / 100.0) * (math.pi / 180.0)
            phi   = (phi_cdeg   / 100.0) * (math.pi / 180.0)
            x = r * math.cos(phi) * math.cos(theta)
            y = r * math.cos(phi) * math.sin(theta)
            z = r * math.sin(phi)
            self._frame_buf.append((x, y, z, int(refl)))
            npts += 1
        return npts

    def _parse_livox_packet(self, raw: bytes) -> None:
        try:
            payload = raw
            plen = len(payload)
            self._last_payload_len = plen
            self._last_pkt_raw = raw
            self._decoded_pts_last = 0
            self._last_stride = None
            self._last_data_type = None
            self._last_header = None
            if plen <= 0:
                return
            npts, used_skip = self._decode_cartesian_high_autoskip(payload)
            if npts > 0:
                self._decoded_pts_last = npts
                self._last_stride = 14
                self._last_header = {"auto_skip": used_skip}
                return
            if (plen % 8) == 0:
                self._decoded_pts_last = self._decode_cartesian_low(payload)
                self._last_stride = 8
                self._last_header = {"auto_skip": 0}
                return
            if (plen % 10) == 0:
                self._decoded_pts_last = self._decode_spherical(payload)
                self._last_stride = 10
                self._last_header = {"auto_skip": 0}
                return
            for tail in (2, 4, 6, 8, 10, 12):
                usable = plen - tail
                if usable > 0 and (usable % 14) == 0:
                    npts, used_skip = self._decode_cartesian_high_autoskip(payload[:usable])
                    if npts > 0:
                        self._decoded_pts_last = npts
                        self._last_stride = 14
                        self._last_header = {"auto_skip": used_skip, "tail_trim": tail}
                        return
            self._decoded_pts_last = 0
            self._last_stride = None
            self._last_header = {"auto_skip": None}
            return
        except Exception as e:
            self.logger.debug("Livox packet decode error: %s", e)
            return

    # ---- IMU decoding ----
    def _parse_imu_packet(self, raw: bytes) -> Optional[Dict[str, float]]:
        for off in range(0, min(64, max(0, len(raw) - 24)) + 1, 2):
            slice_ = raw[off:off + 24]
            if len(slice_) < 24:
                break
            try:
                gx, gy, gz, ax, ay, az = struct.unpack("<6f", slice_)
            except struct.error:
                continue
            if all(map(math.isfinite, (gx, gy, gz, ax, ay, az))) and max(abs(gx), abs(gy), abs(gz)) < 1000.0 and max(abs(ax), abs(ay), abs(az)) < 50.0:
                return {"gx": gx, "gy": gy, "gz": gz, "ax": ax, "ay": ay, "az": az}
        return None

    # ---- run loop ----
    def run(self) -> None:
        self.logger.info("Livox MID-360 run() loop started.")
        last_pub = time.time()
        self._frame_start = last_pub
        try:
            while not self._stop.is_set():
                poll_end = time.time() + 0.01
                while time.time() < poll_end:
                    rlist = [s for s in (self._pt_sock, self._imu_sock) if s]
                    if not rlist:
                        break
                    rs, _, _ = select.select(rlist, [], [], 0.005)
                    for s in rs:
                        try:
                            data, _addr = s.recvfrom(65535)
                            now = time.time()
                            if s is self._pt_sock:
                                self._point_pkts += 1
                                self._point_bytes += len(data)
                                self._last_point_ts = now
                                if self.decode_points:
                                    self._parse_livox_packet(data)
                            else:
                                self._imu_pkts += 1
                                self._imu_bytes += len(data)
                                self._last_imu_ts = now
                                imu = self._parse_imu_packet(data)
                                if imu:
                                    self._imu_last = imu
                        except Exception:
                            pass
                now = time.time()
                should_pub = (now - last_pub) >= self.publish_period
                frame_due  = (now - self._frame_start) >= (self.frame_ms / 1000.0)
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
                    }
                    payload["decode_diag"] = {
                        "data_type": self._last_data_type,
                        "stride": self._last_stride,
                        "payload_len": self._last_payload_len,
                        "decoded_pts_last": self._decoded_pts_last,
                        "header": self._last_header,
                    }
                    if self.decode_points and self._frame_buf:
                        pts = self._frame_buf
                        if len(pts) > self.max_points:
                            step = max(1, len(pts) // self.max_points)
                            pts = pts[::step]
                        pts = self._compact_points(pts)
                        payload["points"] = pts
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
                    self.publish(payload)
                    last_pub = now
                    if frame_due:
                        self._frame_buf = []
                        self._frame_start = now
                time.sleep(0.002)
        except Exception as e:
            self.logger.error("Livox MID-360 run-loop error: %s", e)
        finally:
            self.logger.info("Livox MID-360 run() loop exiting.")

    def _compact_points(self, pts: List[Tuple[float, float, float, int]]) -> List[Tuple]:
        keep = {
            'xy': (0, 1),
            'xyz': (0, 1, 2),
            'xyi': (0, 1, 3),
            'xyzi': (0, 1, 2, 3),
        }.get(self.keep_fields, (0, 1, 2, 3))
        dec = max(0, self.decimals)
        out: List[Tuple] = []
        for (x, y, z, i) in pts:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if max(abs(x), abs(y)) > 50.0 or abs(z) > 10.0:
                continue
            vals = (x, y, z, i)
            row: List[Any] = []
            for idx in keep:
                if idx in (0, 1, 2):
                    row.append(round(vals[idx], dec))
                else:
                    row.append(int(vals[idx]))
            out.append(tuple(row))
        return out

    # ---- Bridge control: IMU enable/disable ----
    def _send_control_json(self, obj: Dict[str, Any]) -> None:
        try:
            msg = json.dumps(obj).encode("utf-8")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
            dst = ("127.0.0.1", self._ctl_port)
            sock.sendto(msg, dst)
            sock.close()
        except Exception as e:
            self.logger.warning("Bridge control send failed: %s", e)

    # =========================
    # NEW: health & readiness
    # =========================
    def _service_state(self) -> Optional[str]:
        """
        Return systemd user unit state string ('active', 'inactive', 'failed', ...)
        or None if not using systemd. Returns 'unknown' on errors.
        """
        if not self.use_systemd:
            return None
        try:
            unit = _unit_name(self.service_name)
            r = _run_systemctl_user(["is-active", unit], capture_output=True, text=True)
            return r.stdout.strip()
        except Exception:
            return "unknown"

    def _last_data_age_sec(self) -> Optional[float]:
        """
        Age (seconds) of the freshest signal (points or IMU).
        None => no data seen yet.
        """
        last = max(self._last_point_ts or 0.0, self._last_imu_ts or 0.0)
        if last <= 0.0:
            return None
        return max(0.0, time.time() - last)

    # Tighten readiness to "has seen reasonably fresh data"
    def is_ready(self) -> bool:
        age = self._last_data_age_sec()
        return (age is not None) and (age < self.warn_stale_sec)

    # Report health for status pages (ok/yellow/red)
    def health(self) -> dict:
        now = time.time()
        age = self._last_data_age_sec()  # None if never saw anything
        svc = self._service_state()      # 'active', 'inactive', 'failed', 'unknown', or None
        running = self.is_running()

        if not running or (svc not in (None, "active")):
            status = "error"
            reason = "adapter not running" if not running else f"service {svc}"
        else:
            if age is None:
                status = "warning"          # bridge up but no data yet
                reason = "no data yet; awaiting packets"
            elif age < self.ok_stale_sec:
                status = "ok"
                reason = "fresh data"
            elif age < self.err_stale_sec:
                status = "warning"
                reason = f"data stale ({age:.1f}s)"
            else:
                status = "error"
                reason = f"no data for {age:.1f}s"

        return {
            "id": self.sensor_id,
            "kind": self.kind,
            "status": status,             # ui: green/ yellow/ red
            "reason": reason,             # short human explanation
            "running": running,
            "service": svc,
            "ready": (status == "ok"),
            "last_point_ts": self._last_point_ts,
            "last_imu_ts": self._last_imu_ts,
            "last_data_age_sec": None if age is None else round(age, 2),
            "point_pkts": self._point_pkts,
            "imu_pkts": self._imu_pkts,
            "timestamp": now,
        }

# Control endpoints to toggle IMU push via bridge (device-side)
@router.post("/imu/enable")
def livox_imu_enable(enable: int = 1):
    try:
        ctl_port = int(os.getenv("LIVOX_CTL_PORT", "18181"))
        msg = json.dumps({"cmd": "set_imu_enable", "enable": int(enable)}).encode("utf-8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        sock.sendto(msg, ("127.0.0.1", ctl_port))
        sock.close()
        return {"ok": True, "cmd": "set_imu_enable", "enable": int(enable), "port": ctl_port}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"IMU enable failed: {e}")

@router.post("/imu/disable")
def livox_imu_disable():
    return livox_imu_enable(enable=0)

@router.get("/debug/peek", response_class=Response)
def livox_debug_peek(adapter_id: str = "livox"):
    body = json.dumps({
        "note": (
            "Use /sensors/livox/latest to inspect 'imu' and 'decode_diag'. "
            "To toggle IMU: POST /livox/imu/enable or /livox/imu/disable."
        )
    }, indent=2)
    return Response(content=body, media_type="application/json")

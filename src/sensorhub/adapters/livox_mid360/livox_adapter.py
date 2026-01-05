
# src/sensorhub/adapters/livox_mid360/livox_adapter.py
"""
Livox MID-360 UDP listener + point decoder (optimized).

This adapter listens to Livox point and IMU UDP streams and publishes a short frame window
including status and optional point data as `(x, y, z, intensity)` in meters.

What's included:
- Systemd service helpers (/livox/service/{status|start|stop}) targeting the **user** systemd.
- UDP multicast join for point and IMU streams.
- Robust Livox Ethernet packet parsing driven by the header `data_type`.
- Headerless-payload heuristic: if a bridge omits the 18-byte LivoxEthPacket header,
  treat the entire packet as payload and attempt known strides (14/8/10 bytes per point)
  to decode Cartesian High / Cartesian Low / Spherical formats.
- Runtime diagnostics published under `decode_diag` in `/sensors/livox/latest`.
- Adapter-side decimation + quantization + field trimming before publishing for lean JSON payloads.
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
from typing import Optional, List, Tuple, Dict, Any

from fastapi import APIRouter, HTTPException, Response
from sensorhub.core.sensor_base import AbstractSensorAdapter

# FastAPI router for optional service controls (user systemd only; no polkit prompts)
router = APIRouter(prefix="/livox", tags=["livox"])

# ---- utility helpers for systemd user commands ----

def _unit_name(name: str) -> str:
    """Ensure the unit has `.service` suffix."""
    return name if name.endswith(".service") else f"{name}.service"

def _run_systemctl_user(args: List[str], **kwargs) -> subprocess.CompletedProcess:
    """
    Run `systemctl --user ...` with absolute path to avoid PATH ambiguities.
    Never uses sudo. Avoids polkit prompts.
    """
    cmd = ["/usr/bin/systemctl", "--user"] + args
    return subprocess.run(cmd, **kwargs)

# ---- service API endpoints (user systemd) ----

@router.get("/service/status")
def livox_service_status(service_name: str = "livox_bridge"):
    try:
        unit = _unit_name(service_name)
        result = _run_systemctl_user(
            ["is-active", unit],
            capture_output=True,
            text=True,
        )
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

class LivoxMid360Adapter(AbstractSensorAdapter):
    """
    Livox MID-360 UDP listener + point decoder.
    - Joins UDP streams (point cloud + IMU) or starts a bridge via systemd (user instance).
    - Aggregates a short frame window and publishes status + optional points in meters:
      publish({"points": [[x, y, z, intensity], ...], ...})

    Data types (per Livox SDK):
      0x00: Cartesian High (int32 mm)
      0x01: Cartesian Low (int16 cm)
      0x02: Spherical (depth + angles)
    """

    # data_type constants
    DATA_TYPE_CARTESIAN_HIGH = 0x00
    DATA_TYPE_CARTESIAN_LOW = 0x01
    DATA_TYPE_SPHERICAL      = 0x02

    def __init__(
        self,
        sensor_id: str,
        kind: str = "lidar3d",
        config_path: str = "/home/dev/livox_configs/mid360_config.json",
        use_systemd: bool = True,
        service_name: str = "livox_bridge",
        bridge_path: Optional[str] = None,
        multicast_ip: str = "224.1.1.5",
        point_port: int = 56301,
        imu_port: int = 56401,
        listen_udp: bool = True,
        publish_period: float = 0.5,
        hz: Optional[float] = None,
        # point decoding options
        decode_points: bool = True,
        frame_ms: int = 100,
        max_points: int = 20000,
        # payload compaction options
        keep_fields: str = "xyzi",   # 'xy' | 'xyz' | 'xyi' | 'xyzi'
        decimals: int = 2,
        **kwargs,
    ) -> None:
        super().__init__(sensor_id, kind)

        self.logger = getattr(
            self,
            "logger",
            logging.getLogger(f"sensorhub.adapters.livox_mid360.{self.__class__.__name__}.{sensor_id}"),
        )
        self._stop = getattr(self, "_stop", threading.Event())

        # Config / systemd / network
        self.config_path = os.path.abspath(config_path)
        self.use_systemd = use_systemd
        self.service_name = service_name
        self.bridge_path = bridge_path
        self.multicast_ip = multicast_ip
        self.point_port = int(point_port)
        self.imu_port = int(imu_port)
        self.listen_udp = bool(listen_udp)

        # publish cadence
        self.publish_period = (1.0 / hz) if (hz and hz > 0) else float(publish_period)

        # sockets/process
        self._pt_sock: Optional[socket.socket] = None
        self._imu_sock: Optional[socket.socket] = None
        self._proc: Optional[subprocess.Popen] = None

        # counters
        self._point_pkts = 0
        self._point_bytes = 0
        self._imu_pkts = 0
        self._imu_bytes = 0
        self._last_point_ts = 0.0
        self._last_imu_ts = 0.0

        # frame + decoding
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

    # ----------------- systemd helpers (user instance) -----------------
    def _systemd_start(self) -> None:
        try:
            unit = _unit_name(self.service_name)
            # Start only if not already active
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

    # ----------------- bridge process (non-systemd) -----------------
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

    # ----------------- UDP sockets -----------------
    def _join_multicast(self, port: int) -> socket.socket:
        # Optional interface binding + larger receive buffer for high-rate streams
        iface = os.getenv('LIVOX_IFACE', '0.0.0.0')
        rcvbuf = int(os.getenv('LIVOX_RCVBUF', '4194304'))  # 4MB default

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

        self.logger.info(
            "Joined multicast %s on UDP port %d (iface=%s, rcvbuf=%d)",
            self.multicast_ip, port, iface, rcvbuf
        )
        return sock

    # ----------------- start/stop -----------------
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

        # Delegate thread management to base class (avoid duplicate threads)
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

    # ----------------- packet decoding -----------------
    def _decode_cartesian_high(self, payload: bytes) -> int:
        """14 bytes per point: int32 x_mm, y_mm, z_mm, u8 reflectivity, u8 tag"""
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

    def _decode_cartesian_high_with_header_heuristic(self, payload: bytes) -> int:
        """
        Decode 14B/pt payload when a small leading/trailing frame header is mixed in.
        Attempts offsets in {0,2,4,6,8,10,12}; if none fit, tries trimming trailing bytes
        by the same set to make length divisible by 14.
        """
        plen = len(payload)

        # Try leading offsets
        for off0 in (0, 2, 4, 6, 8, 10, 12):
            usable = plen - off0
            if usable > 0 and (usable % 14 == 0):
                n = usable // 14
                off = off0
                npts = 0
                for _ in range(n):
                    x_mm, y_mm, z_mm, refl, tag = struct.unpack_from("<iiiBB", payload, off)
                    off += 14
                    self._frame_buf.append((x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, int(refl)))
                    npts += 1
                return npts

        # Try trailing trims
        for tail in (2, 4, 6, 8, 10, 12):
            usable = plen - tail
            if tail < plen and (usable % 14 == 0):
                n = usable // 14
                off = 0
                npts = 0
                for _ in range(n):
                    x_mm, y_mm, z_mm, refl, tag = struct.unpack_from("<iiiBB", payload, off)
                    off += 14
                    self._frame_buf.append((x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, int(refl)))
                    npts += 1
                return npts

        return 0

    def _decode_cartesian_low(self, payload: bytes) -> int:
        """8 bytes per point: int16 x_cm, y_cm, z_cm, u8 reflectivity, u8 tag"""
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
        """
        Minimal spherical converter. The exact packing can vary by firmware; this implementation
        assumes 10 bytes per point: int16 depth_mm, int16 theta_cdeg, int16 phi_cdeg, u8 reflectivity, u8 tag.
        Adjust if your device uses different angular units.
        """
        npts = 0
        stride = 10
        if len(payload) % stride != 0:
            return 0
        n = len(payload) // stride
        off = 0
        for _ in range(n):
            depth_mm, theta_cdeg, phi_cdeg, refl, tag = struct.unpack_from("<hhhBB", payload, off)
            off += stride
            r = depth_mm / 1000.0
            theta = (theta_cdeg / 100.0) * (math.pi / 180.0)
            phi = (phi_cdeg / 100.0) * (math.pi / 180.0)
            x = r * math.cos(phi) * math.cos(theta)
            y = r * math.cos(phi) * math.sin(theta)
            z = r * math.sin(phi)
            self._frame_buf.append((x, y, z, int(refl)))
            npts += 1
        return npts

    def _parse_livox_packet(self, raw: bytes) -> None:
        """
        Decode a Livox Ethernet packet and append points to the frame buffer.
        Robust to bridges that re-pack: if header is missing (<18 bytes), treat entire packet
        as payload and try stride heuristics (14/8/10).
        """
        try:
            # Case A: header present (LivoxEthPacket)
            if len(raw) >= 18:
                version, slot, dev_id, rsvd, err_code, ts_type, data_type = struct.unpack_from("<BBBBIBB", raw, 0)
                payload = raw[18:]
                plen = len(payload)
                self._last_payload_len = plen
                self._last_header = {
                    "version": version,
                    "slot": slot,
                    "id": dev_id,
                    "err_code": err_code,
                    "ts_type": ts_type,
                    "data_type": data_type,
                }
                self._last_pkt_raw = raw
                self._last_data_type = data_type
                self._decoded_pts_last = 0

                if plen <= 0:
                    return

                # Decode by data_type or fallback strides
                if data_type == self.DATA_TYPE_CARTESIAN_HIGH:
                    if plen % 14 == 0:
                        self._decoded_pts_last = self._decode_cartesian_high(payload); self._last_stride = 14
                    else:
                        self._decoded_pts_last = self._decode_cartesian_high_with_header_heuristic(payload); self._last_stride = 14
                elif data_type == self.DATA_TYPE_CARTESIAN_LOW:
                    if plen % 8 == 0:
                        self._decoded_pts_last = self._decode_cartesian_low(payload); self._last_stride = 8
                    else:
                        self._last_stride = 8
                        self._decoded_pts_last = 0
                elif data_type == self.DATA_TYPE_SPHERICAL:
                    if plen % 10 == 0:
                        self._decoded_pts_last = self._decode_spherical(payload); self._last_stride = 10
                    else:
                        self._last_stride = 10
                        self._decoded_pts_last = 0
                else:
                    # Unknown data_type: try strides
                    if plen % 14 == 0:
                        self._decoded_pts_last = self._decode_cartesian_high(payload); self._last_stride = 14
                    elif plen % 8 == 0:
                        self._decoded_pts_last = self._decode_cartesian_low(payload); self._last_stride = 8
                    elif plen % 10 == 0:
                        self._decoded_pts_last = self._decode_spherical(payload); self._last_stride = 10
                    else:
                        # try heuristic high
                        self._decoded_pts_last = self._decode_cartesian_high_with_header_heuristic(payload); self._last_stride = 14
                        if self._decoded_pts_last == 0:
                            self._last_stride = None
                            return

            # Case B: headerless payload (bridge custom framing)
            else:
                payload = raw
                plen = len(payload)
                self._last_payload_len = plen
                self._last_header = None
                self._last_pkt_raw = raw
                self._last_data_type = None
                self._decoded_pts_last = 0

                if plen <= 0:
                    return

                if plen % 14 == 0:
                    self._decoded_pts_last = self._decode_cartesian_high(payload); self._last_stride = 14
                elif plen % 8 == 0:
                    self._decoded_pts_last = self._decode_cartesian_low(payload); self._last_stride = 8
                elif plen % 10 == 0:
                    self._decoded_pts_last = self._decode_spherical(payload); self._last_stride = 10
                else:
                    # try heuristic high
                    self._decoded_pts_last = self._decode_cartesian_high_with_header_heuristic(payload); self._last_stride = 14
                    if self._decoded_pts_last == 0:
                        self._last_stride = None
                        return

            # periodic debug (throttled)
            if self._point_pkts % 500 == 0:
                self.logger.debug(
                    "Livox pkt: header=%s payload=%d decoded_pts=%d stride=%s",
                    "yes" if len(raw) >= 18 else "no", self._last_payload_len, self._decoded_pts_last, str(self._last_stride)
                )

        except Exception as e:
            # decode failure shouldn't crash the adapter; keep status publishing
            self.logger.debug("Livox packet decode error: %s", e)
            return

    def _compact_points(self, pts: List[Tuple[float, float, float, int]]) -> List[Tuple]:
        """Trim fields (xy/xyz/xyi/xyzi) and quantize xyz to reduce payload size."""
        keep = {
            'xy': (0, 1),
            'xyz': (0, 1, 2),
            'xyi': (0, 1, 3),
            'xyzi': (0, 1, 2, 3),
        }.get(self.keep_fields, (0, 1, 2, 3))

        dec = max(0, self.decimals)
        out: List[Tuple] = []
        for (x, y, z, i) in pts:
            vals = (x, y, z, i)
            row: List[Any] = []
            for idx in keep:
                if idx in (0, 1, 2):
                    row.append(round(vals[idx], dec))
                else:
                    row.append(int(vals[idx]))
            out.append(tuple(row))
        return out

    # ----------------- run loop (concrete) -----------------
    def run(self) -> None:
        self.logger.info("Livox MID-360 run() loop started.")
        last_pub = time.time()
        self._frame_start = last_pub
        try:
            while not self._stop.is_set():
                # short poll window for UDP sockets
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
                        except Exception:
                            # ignore transient socket errors
                            pass

                # publish periodically or by frame window
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
                    }

                    # Publish decode diagnostics regardless of points
                    payload["decode_diag"] = {
                        "data_type": self._last_data_type,
                        "stride": self._last_stride,
                        "payload_len": self._last_payload_len,
                        "decoded_pts_last": self._decoded_pts_last,
                        "header": self._last_header,
                    }

                    # Include points if we have any (decimate + compact)
                    if self.decode_points and self._frame_buf:
                        pts = self._frame_buf
                        if len(pts) > self.max_points:
                            step = max(1, len(pts) // self.max_points)
                            pts = pts[::step]
                        pts = self._compact_points(pts)
                        payload["points"] = pts
                        payload["frame_points"] = len(pts)
                        payload["points_format"] = {"data_type": self._last_data_type, "stride": self._last_stride}

                    self.publish(payload)
                    last_pub = now

                    # reset frame window
                    if frame_due:
                        self._frame_buf = []
                        self._frame_start = now

                time.sleep(0.002)  # yield

        except Exception as e:
            self.logger.error("Livox MID-360 run-loop error: %s", e)
        finally:
            self.logger.info("Livox MID-360 run() loop exiting.")

@router.get("/debug/peek", response_class=Response)
def livox_debug_peek(adapter_id: str = "livox"):
    """Minimal debug endpoint. Inspect `decode_diag` via /sensors/livox/latest."""
    import json
    body = json.dumps({
        "note": "Use /sensors/livox/latest to inspect decode_diag: header, data_type, stride, payload_len, decoded_pts_last"
    }, indent=2)
    return Response(content=body, media_type="application/json")

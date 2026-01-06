
# src/sensorhub/adapters/rplidar_s2/rplidar_adapter.py
"""
RPLidar S2 Adapter (USB serial; watchdog reconnect; 360° full revolution publish)

Backends:
 1) SDK bridge: runs Slamtec SDK 'ultra_simple' and parses stdout/stderr lines:
    - Uses SDK 'S' start-flag to delimit full 360° wraps (publish-on-wrap when configured).
    - Angle wrap detection (backup).
 2) Python backend (fallback): iter_measurements → iter_measurments → iter_scans.

References:
- Ultra Simple demo prints 'S' start flag and 'theta/Dist/Q' lines; S2 requires 1,000,000 baud.  # [2](https://superuser.com/questions/1825431/ttyusb-serial-device-not-being-created-in-ubuntu18-04)
- ModemManager commonly probes /dev/ttyUSB* on attach; disable or blacklist CP2102N if binding issues.  # [1](https://www.manualshelf.com/manual/slamtec/rplidar-a2/sdk-manual-english.html)
"""
import os
import re
import time
import threading
import subprocess
import logging
import glob
import pty
import select
import fcntl
from dataclasses import dataclass
from typing import Optional, List, Tuple, Any

from sensorhub.core.sensor_base import AbstractSensorAdapter

# ------------------------- structs -------------------------
@dataclass
class _DeviceInfo:
    model: int
    firmware_version: int
    hardware_version: int
    serial_number: str

@dataclass
class _DeviceHealth:
    status: str
    error_code: int = 0

# ------------------ Python backend ------------------
class _RplidarBackend:
    def __init__(self, port: Optional[str], baudrate: int, timeout: float = 2.5, max_buf_meas: int = 10000):
        from rplidar import RPLidar, RPLidarException  # type: ignore
        self._RPLidar = RPLidar
        self._RPLidarException = RPLidarException

        self._requested_port = port or "/dev/ttyUSB0"
        self._port = self._requested_port
        self._baud_candidates = [baudrate, 115200, 256000, 460800, 1000000]
        self._baudrate = baudrate
        self._timeout = timeout
        self._max_buf_meas = int(max_buf_meas)

        self._lidar: Optional[RPLidar] = None  # type: ignore
        self._mode: str = "unknown"
        self._meas_iter = None
        self._scan_iter = None
        self._connected = False
        self._scanning = False
        self._log = logging.getLogger(__name__)

    def _select_usb_port(self) -> str:
        if self._requested_port and os.path.exists(self._requested_port):
            return self._requested_port
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        if by_id:
            self._log.warning("Requested port '%s' not found; trying '%s'", self._requested_port, by_id[0])
            return by_id[0]
        candidates = sorted(glob.glob("/dev/ttyUSB*"))
        if candidates:
            self._log.warning("Requested port '%s' not found; trying '%s'", self._requested_port, candidates[0])
            return candidates[0]
        return self._requested_port

    def _open_port(self, port: str, baud: int) -> Optional[Any]:
        try:
            import logging as _logging
            quiet_logger = _logging.getLogger(f"sensorhub.rplidar.quiet.{port}.{baud}")
            quiet_logger.propagate = False
            if not quiet_logger.handlers:
                h = _logging.StreamHandler()
                h.setFormatter(_logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s"))
                quiet_logger.addHandler(h)
            quiet_logger.setLevel(_logging.WARNING)
            dev = self._RPLidar(port, baudrate=baud, timeout=self._timeout, logger=quiet_logger)
            try: dev.clear_input()
            except Exception: pass
            return dev
        except Exception as e:
            self._log.error("Open port failed (%s@%d): %s", port, baud, e)
            return None

    def connect(self) -> bool:
        self._port = os.path.realpath(self._select_usb_port())
        for b in self._baud_candidates:
            dev = self._open_port(self._port, b)
            if dev is None:
                continue
            self._lidar = dev
            self._baudrate = b
            self._connected = True
            self._log.info("RPLidar connected on %s@%d (timeout=%.1f)", self._port, self._baudrate, self._timeout)
            # Optional: info/health/reset
            try:
                info = self.get_device_info()
                if info:
                    self._log.info("RPLidar info: model=%d fw=%d hw=%d serial=%s",
                                   info.model, info.firmware_version, info.hardware_version, info.serial_number)
                health = self.get_health()
                if health and str(health.status).lower() == "error":
                    self._log.warning("Health ERROR; attempting reset()")
                    try:
                        self._lidar.reset()  # type: ignore
                        time.sleep(1.0)
                        try: self._lidar.clear_input()
                        except Exception: pass
                    except Exception as e:
                        self._log.error("Reset failed: %s", e)
            except Exception:
                pass
            return True
        self._lidar = None
        self._connected = False
        return False

    def disconnect(self) -> None:
        if self._lidar is not None:
            try: self.stop_scan()
            except Exception: pass
            try: self._lidar.disconnect()
            except Exception: pass
        self._lidar = None
        self._connected = False

    def get_device_info(self) -> Optional[_DeviceInfo]:
        if not self._connected or self._lidar is None:
            return None
        try:
            info = self._lidar.get_info()
            return _DeviceInfo(
                model=int(info.get("model", 0)),
                firmware_version=int(info.get("firmware", 0)),
                hardware_version=int(info.get("hardware", 0)),
                serial_number=str(info.get("serialnumber", "")),
            )
        except Exception:
            return None

    def get_health(self) -> Optional[_DeviceHealth]:
        if not self._connected or self._lidar is None:
            return None
        try:
            status, error_code = self._lidar.get_health()
            return _DeviceHealth(status=status, error_code=int(error_code))
        except Exception:
            return None

    def start_scan(self) -> bool:
        if not self._connected or self._lidar is None:
            return False
        try:
            try: self._lidar.start_motor()
            except Exception: pass
            time.sleep(0.5)
            try: self._lidar.start()
            except Exception: pass
            time.sleep(0.5)
            try: self._lidar.clear_input()
            except Exception: pass

            has_iter_measurements = hasattr(self._lidar, "iter_measurements")
            has_iter_measurments = hasattr(self._lidar, "iter_measurments")
            has_iter_scans = hasattr(self._lidar, "iter_scans")

            if has_iter_measurements:
                try:
                    self._meas_iter = getattr(self._lidar, "iter_measurements")(max_buf_meas=self._max_buf_meas)
                    self._mode = "measurements"; self._scanning = True
                    self._log.info("RPLidar: iter_measurements(max_buf_meas=%d)", self._max_buf_meas)
                    return True
                except Exception as e:
                    self._log.debug("iter_measurements() failed: %s", e)

            if has_iter_measurments:
                try:
                    self._meas_iter = getattr(self._lidar, "iter_measurments")(max_buf_meas=self._max_buf_meas)
                    self._mode = "measurments"; self._scanning = True
                    self._log.info("RPLidar: iter_measurments(max_buf_meas=%d)", self._max_buf_meas)
                    return True
                except Exception as e:
                    self._log.debug("iter_measurments() failed: %s", e)

            if has_iter_scans:
                try:
                    self._scan_iter = getattr(self._lidar, "iter_scans")(
                        scan_type="standard", max_buf_meas=self._max_buf_meas, min_len=5
                    )
                    self._mode = "scans"; self._scanning = True
                    self._log.info("RPLidar: iter_scans(standard, max_buf_meas=%d)", self._max_buf_meas)
                    return True
                except TypeError:
                    try:
                        self._scan_iter = getattr(self._lidar, "iter_scans")()
                        self._mode = "scans"; self._scanning = True
                        self._log.info("RPLidar: iter_scans() no-kwargs")
                        return True
                    except Exception as e:
                        self._log.debug("iter_scans() no-kwargs failed: %s", e)
                except Exception as e:
                    self._log.debug("iter_scans(standard, ...) failed: %s", e)

                try:
                    self._scan_iter = getattr(self._lidar, "iter_scans")(
                        scan_type="express", max_buf_meas=self._max_buf_meas, min_len=5
                    )
                    self._mode = "scans"; self._scanning = True
                    self._log.info("RPLidar: iter_scans(express, max_buf_meas=%d)", self._max_buf_meas)
                    return True
                except Exception as e:
                    self._log.debug("iter_scans(express, ...) failed: %s", e)

                try:
                    self._scan_iter = getattr(self._lidar, "iter_scans")(max_buf_meas=self._max_buf_meas)
                    self._mode = "scans"; self._scanning = True
                    self._log.info("RPLidar: iter_scans(max_buf_meas=%d)", self._max_buf_meas)
                    return True
                except TypeError:
                    self._scan_iter = getattr(self._lidar, "iter_scans")()
                    self._mode = "scans"; self._scanning = True
                    self._log.info("RPLidar: iter_scans() fallback no-kwargs")
                    return True
                except Exception as e:
                    self._log.debug("iter_scans(max_buf_meas=..) failed: %s", e)

            self._log.error("No supported iterator available on RPLidar object")
            self._meas_iter = None; self._scan_iter = None; self._scanning = False
            return False

        except Exception as e:
            self._log.error("start_scan exception: %s", e)
            self._meas_iter = None; self._scan_iter = None; self._scanning = False
            return False

    def stop_scan(self) -> None:
        if self._lidar is not None:
            try: self._lidar.stop()
            except Exception: pass
            try: self._lidar.stop_motor()
            except Exception: pass
        self._scanning = False
        self._meas_iter = None
        self._scan_iter = None

    def get_scan_data(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        if not self._scanning:
            return None

        if self._mode in ("measurements", "measurments") and self._meas_iter is not None:
            q: List[int] = []; a: List[float] = []; d: List[float] = []
            started = False; t0 = time.time()
            try:
                while True:
                    new_scan, quality, angle, dist = next(self._meas_iter)
                    if quality is None: quality = 0
                    if new_scan and started: break
                    started = True
                    q.append(int(quality)); a.append(float(angle)); d.append(float(dist))
                    if (time.time() - t0) > 0.25 and len(a) > 10: break
            except StopIteration: return None
            except Exception: return None
            return (a, d, q)

        if self._mode == "scans" and self._scan_iter is not None:
            try:
                scan = next(self._scan_iter)  # list of (quality, angle, distance)
                q: List[int] = []; a: List[float] = []; d: List[float] = []
                for quality, angle, dist in scan:
                    q.append(int(quality)); a.append(float(angle)); d.append(float(dist))
                return (a, d, q)
            except StopIteration: return None
            except Exception: return None

        return None

# ------------------ SDK bridge backend ------------------
class _SDKBridgeBackend:
    _LINE_RE = re.compile(
        r"^\s*(S\s+)?theta:\s*([0-9]+(?:\.[0-9]+)?)\s+"
        r"Dist:\s*([0-9]+(?:\.[0-9]+)?)\s+Q:\s*(\d+)\s*$",
        re.IGNORECASE,
    )
    ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

    def __init__(
        self,
        ultra_simple_path: str,
        port: str,
        baud: int,
        poll_interval: float = 0.01,
        wrap_threshold_deg: float = 5.0,
        use_channel_args: bool = True,
        force_line_buffering: bool = True,
        use_pty: bool = False,
        prefer_pty: bool = False,
        max_revolution_ms: float = 1000.0,
        debug_log_first_lines: int = 0,
        launch_timeout_sec: float = 10.0,
        echo_stdout: bool = False,
        log_parsed_triples: bool = False,
        strict_parse_only: bool = True,
        fastpath_after_handshake: bool = True,
        require_wrap_for_publish: bool = False,
    ):
        self._path = str(ultra_simple_path)
        self._port = str(port)
        self._baud = int(baud)
        self._poll = float(poll_interval)
        self._wrap = float(wrap_threshold_deg)
        self._use_channel = bool(use_channel_args)
        self._force_linebuf = bool(force_line_buffering)
        self._use_pty = bool(use_pty)
        self._prefer_pty = bool(prefer_pty)
        self._max_rev_ms = float(max_revolution_ms)
        self._debug_lines_left = int(debug_log_first_lines)
        self._launch_timeout_sec = float(launch_timeout_sec)
        self._echo = bool(echo_stdout)
        self._log_parsed = bool(log_parsed_triples)
        self._strict_only = bool(strict_parse_only)
        self._fastpath_enabled = bool(fastpath_after_handshake); self._fastpath = False
        self._require_wrap = bool(require_wrap_for_publish)

        self._proc: Optional[subprocess.Popen] = None
        self._stdout = None; self._stderr = None
        self._pty_master_fd: Optional[int] = None
        self._pty_child: bool = False; self._pty_buf: str = ""

        self._a: List[float] = []; self._r: List[float] = []; self._q: List[int] = []
        self._last_ang: Optional[float] = None
        self._rev_start_ts: float = 0.0

        self._connected = False; self._scanning = False
        self._attempted_port_retry: bool = False
        self._log = logging.getLogger(__name__)

    def _select_usb_port(self) -> str:
        req = (self._port or "").strip()
        if req and os.path.exists(req) and "<your-device-id>" not in req:
            return req
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        if by_id:
            self._log.warning("Requested port '%s' not found; using '%s'", req or "<empty>", by_id[0])
            return by_id[0]
        usb = sorted(glob.glob("/dev/ttyUSB*"))
        if usb:
            self._log.warning("Requested port '%s' not found; using '%s'", req or "<empty>", usb[0])
            return usb[0]
        return req or "/dev/ttyUSB0"

    def connect(self) -> bool:
        if not os.path.isfile(self._path) or not os.access(self._path, os.X_OK):
            self._log.error("SDK ultra_simple not found or not executable: %s", self._path)
            self._connected = False; return False
        self._port = os.path.realpath(self._select_usb_port())
        self._connected = True
        return True

    def _base_args(self) -> List[str]:
        return ["--channel", "--serial", self._port, str(self._baud)] if self._use_channel else [self._port, str(self._baud)]

    def _cmd_stdbuf(self) -> List[str]:
        base = [self._path] + self._base_args()
        return ["stdbuf", "-oL", "-eL"] + base if self._force_linebuf else base

    def _launch_stdbuf(self) -> bool:
        try:
            cmd = self._cmd_stdbuf()
            self._log.info("SDK ultra_simple launching (stdbuf): %s", " ".join(cmd))
            env = os.environ.copy()
            env.update({"TERM": "dumb", "LC_ALL": "C", "LANG": "C"})
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env,
            )
            self._stdout = self._proc.stdout
            self._stderr = self._proc.stderr
            self._pty_child = False
            return True
        except Exception as e:
            self._log.error("stdbuf launch failed: %s", e)
            self._proc = None; self._stdout = None; self._stderr = None
            return False

    def _launch_pty(self) -> bool:
        try:
            master_fd, slave_fd = pty.openpty()
            cmd = [self._path] + self._base_args()
            self._log.info("SDK ultra_simple launching (PTY): %s", " ".join(cmd))
            self._proc = subprocess.Popen(
                cmd,
                stdin=slave_fd,
                stdout=slave_fd,
                stderr=slave_fd,
                text=False,
                bufsize=0,
                close_fds=True
            )
            os.close(slave_fd)
            fcntl.fcntl(master_fd, fcntl.F_SETFL, os.O_NONBLOCK)
            self._pty_master_fd = master_fd
            self._stdout = None; self._stderr = None
            self._pty_child = True; self._pty_buf = ""
            return True
        except Exception as e:
            self._log.error("PTY launch failed: %s", e)
            self._proc = None; self._pty_master_fd = None
            return False

    def _retry_with_fallback_port(self) -> bool:
        if self._attempted_port_retry:
            return False
        self._attempted_port_retry = True

        fallback = "/dev/ttyUSB0"
        if not os.path.exists(fallback):
            fallback = self._select_usb_port()
        self._port = os.path.realpath(fallback)
        self._log.warning("SDK bridge retrying with fallback port: %s", self._port)

        self.stop_scan()
        if not self._launch_stdbuf():
            if self._use_pty and not self._launch_pty():
                return False

        t0 = time.time()
        while (time.time() - t0) < self._launch_timeout_sec:
            line = self._read_line_once()
            if line:
                if "Usage:" in line or "Error" in line or "cannot bind to the specified serial port" in line:
                    self._log.error("SDK bridge error on fallback: %s", line.strip())
                    return False
                if self._parse_line(line, mark_handshake=True):
                    self._scanning = True
                    return True
            time.sleep(0.05)
        return False

    def start_scan(self) -> bool:
        if not self._connected:
            return False

        if self._prefer_pty and self._use_pty:
            if not self._launch_pty():
                return False
            t0 = time.time()
            while (time.time() - t0) < self._launch_timeout_sec:
                line = self._read_line_once()
                if line:
                    if "Usage:" in line or "Error" in line or "cannot bind to the specified serial port" in line:
                        self._log.error("SDK bridge (PTY) reported: %s", line.strip())
                        return self._retry_with_fallback_port()
                    if self._parse_line(line, mark_handshake=True):
                        self._scanning = True; return True
                time.sleep(0.05)
            self._log.error("SDK bridge (PTY): no parsable output"); return False

        if not self._launch_stdbuf():
            if self._use_pty and not self._launch_pty():
                return False

        t0 = time.time()
        while (time.time() - t0) < self._launch_timeout_sec:
            line = self._read_line_once()
            if line:
                if "Usage:" in line or "Error" in line or "cannot bind to the specified serial port" in line:
                    self._log.error("SDK bridge reported: %s", line.strip())
                    return self._retry_with_fallback_port()
                if self._parse_line(line, mark_handshake=True):
                    self._scanning = True; return True
            time.sleep(0.05)

        if not self._pty_child and self._use_pty:
            self._log.warning("No parsable output from stdbuf; switching to PTY")
            self.stop_scan()
            if not self._launch_pty():
                return False
            t1 = time.time()
            while (time.time() - t1) < self._launch_timeout_sec:
                line = self._read_line_once()
                if line:
                    if "Usage:" in line or "Error" in line or "cannot bind to the specified serial port" in line:
                        self._log.error("SDK bridge (PTY) reported: %s", line.strip())
                        return self._retry_with_fallback_port()
                    if self._parse_line(line, mark_handshake=True):
                        self._scanning = True; return True
                time.sleep(0.05)

        self._log.error("SDK bridge: no parsable output")
        return False

    def stop_scan(self) -> None:
        if self._proc:
            try:
                self._proc.terminate()
                try: self._proc.wait(timeout=2.0)
                except Exception: self._proc.kill()
            except Exception: pass
        if self._pty_master_fd is not None:
            try: os.close(self._pty_master_fd)
            except Exception: pass
        self._proc = None; self._stdout = None; self._stderr = None
        self._pty_master_fd = None; self._scanning = False
        self._a.clear(); self._r.clear(); self._q.clear()
        self._last_ang = None; self._rev_start_ts = 0.0
        self._pty_buf = ""; self._fastpath = False

    def _clean_line(self, s: str) -> str:
        return self.ANSI_RE.sub("", s).replace("\r", "")

    def _parse_line(self, line: str, mark_handshake: bool = False) -> Optional[Tuple[bool, float, float, int]]:
        m = self._LINE_RE.search(line)
        if not m:
            if not self._strict_only:
                m1 = re.search(r"theta:\s*([0-9]+(?:\.[0-9]+)?)", line, re.IGNORECASE)
                m2 = re.search(r"Dist:\s*([0-9]+(?:\.[0-9]+)?)", line, re.IGNORECASE)
                m3 = re.search(r"Q:\s*(\d+)", line, re.IGNORECASE)
                if not (m1 and m2 and m3): return None
                try:
                    ang = float(m1.group(1)); dist = float(m2.group(1)); qual = int(m3.group(1))
                    if mark_handshake and self._fastpath_enabled: self._fastpath = True
                    return False, ang, dist, qual
                except Exception:
                    return None
            return None

        try:
            start = bool(m.group(1))
            ang = float(m.group(2)); dist = float(m.group(3)); qual = int(m.group(4))
            if mark_handshake and self._fastpath_enabled: self._fastpath = True
            return start, ang, dist, qual
        except Exception:
            return None

    def _is_wrap(self, ang: float) -> bool:
        return self._last_ang is not None and (ang + self._wrap) < self._last_ang

    def _flush_if_ready(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        if len(self._a) >= 4:
            out = (self._a[:], self._r[:], self._q[:])
            self._a.clear(); self._r.clear(); self._q.clear()
            self._last_ang = None; self._rev_start_ts = 0.0
            return out
        return None

    def _maybe_publish_partial(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        if self._require_wrap:
            return None
        if self._rev_start_ts == 0.0:
            return None
        if (time.time() - self._rev_start_ts) * 1000.0 >= self._max_rev_ms and len(self._a) >= 4:
            return self._flush_if_ready()
        return None

    def _read_line_once(self) -> Optional[str]:
        try:
            if self._pty_child and self._pty_master_fd is not None:
                rlist, _, _ = select.select([self._pty_master_fd], [], [], 0)
                if rlist:
                    data = os.read(self._pty_master_fd, 4096)
                    if not data: return None
                    self._pty_buf += data.decode(errors="ignore")
                    if "\n" in self._pty_buf:
                        lines = self._pty_buf.splitlines()
                        if not self._pty_buf.endswith("\n"):
                            self._pty_buf = lines[-1]; lines = lines[:-1]
                        else:
                            self._pty_buf = ""
                        for ln in lines:
                            cln = self._clean_line(ln)
                            if ("theta:" in cln and "Dist:" in cln and "Q:" in cln) or "Usage:" in cln or "Error" in cln or "cannot bind to the specified serial port" in cln:
                                return cln
                        if lines:
                            ln = lines[-1]
                            cln = self._clean_line(ln)
                            return cln
                    return None
            else:
                if self._stdout:
                    ln = self._stdout.readline()
                    if ln: return self._clean_line(ln)
                if self._stderr:
                    le = self._stderr.readline()
                    if le: return self._clean_line(le)
                return None
        except Exception:
            return None

    def get_scan_data(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        if not self._scanning: return None

        line = self._read_line_once()
        if not line:
            part = self._maybe_publish_partial()
            return part if part else None

        if "Usage:" in line or "Error" in line or "cannot bind to the specified serial port" in line:
            return None

        parsed = self._parse_line(line)
        if not parsed: return None

        start_flag, ang, dist_mm, qual = parsed
        now = time.time()

        if start_flag:
            flushed = self._flush_if_ready()
            self._a.clear(); self._r.clear(); self._q.clear()
            self._rev_start_ts = now
            self._last_ang = None
            if flushed:
                return flushed

        if self._rev_start_ts == 0.0:
            self._rev_start_ts = now

        if self._is_wrap(ang):
            flushed = self._flush_if_ready()
            self._rev_start_ts = now
            self._last_ang = None
            if flushed:
                return flushed

        self._a.append(ang); self._r.append(dist_mm); self._q.append(qual)
        self._last_ang = ang
        part = self._maybe_publish_partial()
        return part if part else None

# -------------------------- Adapter --------------------------
class RPLidarS2Adapter(AbstractSensorAdapter):
    def __init__(
        self,
        sensor_id: str,
        kind: str = "lidar2d",
        port: Optional[str] = "/dev/ttyUSB0",
        baud: int = 115200,
        hz: Optional[float] = None,
        publish_empty_scans: bool = False,
        startup_delay: float = 0.8,
        poll_interval: float = 0.01,
        min_points: int = 180,
        empty_log_interval: float = 2.0,
        no_data_reconnect_sec: float = 8.0,
        backoff_initial: float = 1.0,
        backoff_max: float = 5.0,
        max_points: int = 720,
        timeout: float = 2.5,
        use_sdk_bridge: bool = True,
        ultra_simple_path: str = "/home/dev/rplidar_sdk/output/Linux/Release/ultra_simple",
        wrap_threshold_deg: float = 5.0,
        use_channel_args: bool = True,
        force_line_buffering: bool = True,
        use_pty: bool = False,
        prefer_pty: bool = False,
        max_revolution_ms: float = 1000.0,
        debug_log_first_lines: int = 0,
        max_buf_meas: int = 10000,
        drain_scans_per_tick: int = 1,
        fallback_to_python_on_sdk_failure: bool = True,
        launch_timeout_sec: float = 10.0,
        echo_stdout: bool = False,
        log_parsed_triples: bool = False,
        strict_parse_only: bool = True,
        fastpath_after_handshake: bool = True,
        require_wrap_for_publish: bool = False,
        enable_latest_raw_cache: bool = True,
    ):
        super().__init__(sensor_id, kind)
        self.logger = getattr(
            self, "logger",
            logging.getLogger(f"sensorhub.adapters.rplidar_s2.{self.__class__.__name__}.{sensor_id}")
        )
        self._stop = getattr(self, "_stop", threading.Event())

        self.port = port or "/dev/ttyUSB0"
        self.baud = int(baud)
        self.timeout = float(timeout)
        self.hz = hz
        self.publish_empty_scans = bool(publish_empty_scans)

        self._startup_delay = float(startup_delay)
        self._poll_interval = float(poll_interval)
        self._min_points = int(min_points)
        self._empty_log_interval = float(empty_log_interval)
        self._no_data_reconnect_sec = float(no_data_reconnect_sec)
        self._backoff_initial = float(backoff_initial)
        self._backoff_max = float(backoff_max)
        self._backoff_sec = self._backoff_initial
        self._max_points = int(max_points)

        self._use_sdk_bridge = bool(use_sdk_bridge)
        self._ultra_simple_path = str(ultra_simple_path)
        self._wrap_threshold_deg = float(wrap_threshold_deg)
        self._use_channel_args = bool(use_channel_args)
        self._force_linebuf = bool(force_line_buffering)
        self._use_pty = bool(use_pty)
        self._prefer_pty = bool(prefer_pty)
        self._max_revolution_ms = float(max_revolution_ms)
        self._debug_log_first_lines = int(debug_log_first_lines)
        self._max_buf_meas = int(max_buf_meas)
        self._drain_scans_per_tick = int(drain_scans_per_tick)
        self._fallback_to_python = bool(fallback_to_python_on_sdk_failure)
        self._sdk_launch_timeout_sec = float(launch_timeout_sec)
        self._echo_stdout = bool(echo_stdout)
        self._log_parsed_triples = bool(log_parsed_triples)
        self._strict_parse_only = bool(strict_parse_only)
        self._fastpath_after_handshake = bool(fastpath_after_handshake)
        self._require_wrap_for_publish = bool(require_wrap_for_publish)
        self._enable_latest_raw_cache = bool(enable_latest_raw_cache)

        self._driver = None
        self._last_empty_log = 0.0
        self._last_pub_ts: float = 0.0
        self._min_pub_period: float = (1.0 / hz) if (hz and hz > 0) else 0.0
        self._last_scan_ts: float = 0.0
        self._reconnecting: bool = False

        self._latest_frame: Optional[dict] = None

    def get_latest_frame(self) -> Optional[dict]:
        return self._latest_frame

    def start(self):
        backend_name = "SDKBridge" if self._use_sdk_bridge else "PythonRplidar"
        transport = f"{self.port}@{self.baud}"
        self.logger.info("RPLIDAR S2 connecting via %s (hz=%s, backend=%s)...", transport, self.hz, backend_name)

        self._driver = self._make_backend()
        if not self._driver.connect():
            raise RuntimeError(f"RPLIDAR S2 connect failed ({transport}, backend={backend_name})")

        if not self._driver.start_scan():
            self.logger.error("start_scan failed (backend=%s)", backend_name)
            if self._use_sdk_bridge and self._fallback_to_python:
                self.logger.warning("Bridge failed; falling back to Python driver.")
                try:
                    if self._driver:
                        try: self._driver.stop_scan()
                        except Exception: pass
                        try:
                            if hasattr(self._driver, "disconnect"):
                                self._driver.disconnect()  # type: ignore
                        except Exception: pass
                    self._use_sdk_bridge = False
                    backend_name = "PythonRplidar"
                    self._driver = self._make_backend()
                    if not (self._driver.connect() and self._driver.start_scan()):
                        raise RuntimeError("Python backend start_scan failed after bridge failure")
                except Exception as e:
                    raise RuntimeError(f"RPLIDAR S2 start_scan failed; fallback also failed: {e}")
            else:
                raise RuntimeError("RPLIDAR S2 start_scan failed")

        time.sleep(self._startup_delay)
        self._last_scan_ts = time.time()
        super().start()

    def stop(self):
        try:
            if self._driver:
                try: self._driver.stop_scan()
                except Exception: pass
                try:
                    if hasattr(self._driver, "disconnect"):
                        self._driver.disconnect()  # type: ignore
                except Exception: pass
        finally:
            super().stop()

    def _make_backend(self):
        if self._use_sdk_bridge:
            return _SDKBridgeBackend(
                ultra_simple_path=self._ultra_simple_path,
                port=self.port,
                baud=self.baud,
                poll_interval=self._poll_interval,
                wrap_threshold_deg=self._wrap_threshold_deg,
                use_channel_args=self._use_channel_args,
                force_line_buffering=self._force_linebuf,
                use_pty=self._use_pty,
                prefer_pty=self._prefer_pty,
                max_revolution_ms=self._max_revolution_ms,
                debug_log_first_lines=self._debug_log_first_lines,
                launch_timeout_sec=self._sdk_launch_timeout_sec,
                echo_stdout=self._echo_stdout,
                log_parsed_triples=self._log_parsed_triples,
                strict_parse_only=self._strict_parse_only,
                fastpath_after_handshake=self._fastpath_after_handshake,
                require_wrap_for_publish=self._require_wrap_for_publish,
            )
        return _RplidarBackend(self.port, self.baud, timeout=self.timeout, max_buf_meas=self._max_buf_meas)

    def _decimate(self, a: List[float], r: List[float], q: List[int]) -> Tuple[List[float], List[float], List[int]]:
        n = len(a)
        if n <= self._max_points: return a, r, q
        step = max(1, n // self._max_points)
        return a[::step], r[::step], q[::step]

    def _should_publish(self, now: float) -> bool:
        return self._min_pub_period == 0.0 or (now - self._last_pub_ts) >= self._min_pub_period

    def _schedule_reconnect_if_needed(self, now: float) -> None:
        if (self._no_data_reconnect_sec > 0
            and (now - self._last_scan_ts) >= self._no_data_reconnect_sec
            and not self._reconnecting):
            self.logger.warning("RPLIDAR S2: no data for %.1fs; attempting reconnect", self._no_data_reconnect_sec)
            self._safe_reconnect()

    def _safe_reconnect(self):
        self._reconnecting = True
        try:
            backoff = self._backoff_initial
            while not self._stop.is_set():
                try:
                    if self._driver:
                        try: self._driver.stop_scan()
                        except Exception: pass
                        try:
                            if hasattr(self._driver, "disconnect"):
                                self._driver.disconnect()  # type: ignore
                        except Exception: pass
                    self._driver = self._make_backend()
                    if self._driver.connect() and self._driver.start_scan():
                        self.logger.info("RPLIDAR S2: reconnected and scanning.")
                        time.sleep(self._startup_delay)
                        self._last_scan_ts = time.time()
                        self._backoff_sec = self._backoff_initial
                        break
                    else:
                        raise RuntimeError("connect/start_scan failed during reconnect")
                except Exception as e:
                    self.logger.error("RPLIDAR S2 reconnect failed: %s; retry in %.1fs", e, backoff)
                    time.sleep(backoff); backoff = min(backoff * 2, self._backoff_max)
        finally:
            self._reconnecting = False

    def run(self):
        self.logger.info("RPLIDAR S2 run() loop started.")
        try:
            while not self._stop.is_set():
                now = time.time()
                drained = 0
                max_scans_per_tick = self._drain_scans_per_tick

                while not self._stop.is_set() and drained < max_scans_per_tick:
                    scan = None
                    try:
                        scan = self._driver.get_scan_data() if self._driver else None
                    except Exception as e:
                        self.logger.warning("RPLIDAR S2 read error: %s", e)
                        break

                    if not scan: break
                    a, r, q = scan
                    now = time.time(); self._last_scan_ts = now

                    # --- NEW: always cache latest_raw on ANY full frame ---
                    if self._enable_latest_raw_cache and len(a) >= 4:
                        a2, r2, q2 = self._decimate(a, r, q)
                        self._latest_frame = {
                            "sensor_id": self.sensor_id,
                            "angles": a2, "ranges": r2, "qualities": q2,
                            "timestamp": now,
                            "partial": False,  # full wrap path
                        }

                    # Publish when gates satisfied
                    if len(a) >= self._min_points and self._should_publish(now):
                        a2, r2, q2 = self._decimate(a, r, q)
                        frame = {"sensor_id": self.sensor_id, "angles": a2, "ranges": r2, "qualities": q2, "timestamp": now}
                        self.publish(frame)
                        if self._enable_latest_raw_cache:
                            self._latest_frame = frame
                        try:
                            from sensorhub.core.sensor_manager import manager
                            if hasattr(manager, "set_latest_sample"):
                                manager.set_latest_sample(self.sensor_id, frame)
                            elif hasattr(manager, "latest_samples"):
                                manager.latest_samples[self.sensor_id] = frame
                        except Exception:
                            pass

                        self._last_pub_ts = now
                        self._backoff_sec = self._backoff_initial
                        drained += 1

                if drained == 0:
                    if self.publish_empty_scans and (now - self._last_empty_log) >= self._empty_log_interval:
                        self.logger.info("RPLIDAR S2: get_scan_data() returned None")
                        self._last_empty_log = now
                    self._schedule_reconnect_if_needed(now)

                time.sleep(self._poll_interval)

        except Exception as e:
            self.logger.error("RPLIDAR S2 run-loop error: %s", e)
        finally:
            self.logger.info("RPLIDAR S2 run() loop exiting.")

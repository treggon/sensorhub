
# src/sensorhub/adapters/rplidar_s2/rplidar_adapter.py
"""
RPLidar S2 Adapter (USB serial; thread-safe; watchdog reconnect)
Preserves original module/class:
  module: sensorhub.adapters.rplidar_s2.rplidar_adapter
  class : RPLidarS2Adapter

Backends:
  1) SDK bridge backend: runs Slamtec SDK 'ultra_simple' and parses stdout:
     - Launch with '--channel --serial <port> <baud>'
     - Optional 'stdbuf -oL -eL' (pipes) or PTY (pseudo-terminal)
     - STRICT HANDSHAKE: mark scanning only after a parsable "theta ... Dist ... Q ..." line.
     - Partial-scan publishing if a revolution takes too long.

  2) Python 'rplidar-roboticia' (or compatible) backend:
     - Feature-detect: iter_measurements -> iter_measurments -> iter_scans
     - Try STANDARD scan mode first for S2, then EXPRESS, then bare iter_scans().
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


# ----------------------------- structs ---------------------------------------
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


# ---------------------- Python rplidar backend (USB serial) -------------------
class _RplidarBackend:
    """
    Python backend for RPLidar S2 with feature detection:
      - iter_measurements (preferred)
      - iter_measurments (legacy spelling)
      - iter_scans (fallback; try STANDARD first; then EXPRESS; then bare)
    """
    def __init__(self, port: Optional[str], baudrate: int, timeout: float = 2.5, max_buf_meas: int = 10000):
        # Lazy import to avoid errors at module import time
        import rplidar  # type: ignore
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
        self._mode: str = "unknown"  # 'measurements' | 'measurments' | 'scans'
        self._meas_iter = None
        self._scan_iter = None
        self._connected = False
        self._scanning = False
        self._log = logging.getLogger(__name__)

    def _select_usb_port(self) -> str:
        if os.path.exists(self._requested_port):
            return self._requested_port
        candidates = sorted(glob.glob("/dev/ttyUSB*"))
        if candidates:
            self._log.warning(
                "Requested port '%s' not found; trying '%s'",
                self._requested_port, candidates[0]
            )
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
            try:
                dev.clear_input()
            except Exception:
                pass
            return dev
        except Exception as e:
            self._log.error("Open port failed (%s@%d): %s", port, baud, e)
            return None

    def connect(self) -> bool:
        self._port = self._select_usb_port()
        for b in self._baud_candidates:
            dev = self._open_port(self._port, b)
            if dev is None:
                continue
            self._lidar = dev
            self._baudrate = b
            self._connected = True
            self._log.info("RPLidar connected on %s@%d (timeout=%.1f)", self._port, self._baudrate, self._timeout)
            return True
        self._lidar = None
        self._connected = False
        return False

    def disconnect(self) -> None:
        if self._lidar is not None:
            try:
                self.stop_scan()
            except Exception:
                pass
            try:
                self._lidar.disconnect()
            except Exception:
                pass
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
            # Explicit start sequence (helps on some S2 forks)
            try:
                self._lidar.start_motor()
            except Exception:
                pass
            time.sleep(0.4)
            try:
                self._lidar.start()
            except Exception:
                pass
            time.sleep(0.6)
            try:
                self._lidar.clear_input()
            except Exception:
                pass

            # Feature-detect iterators
            has_iter_measurements = hasattr(self._lidar, "iter_measurements")
            has_iter_measurments = hasattr(self._lidar, "iter_measurments")  # legacy spelling
            has_iter_scans = hasattr(self._lidar, "iter_scans")

            # measurement modes first
            if has_iter_measurements:
                try:
                    self._meas_iter = getattr(self._lidar, "iter_measurements")(max_buf_meas=self._max_buf_meas)
                    self._mode = "measurements"
                    self._scanning = True
                    self._log.info("RPLidar: using iter_measurements(max_buf_meas=%d)", self._max_buf_meas)
                    return True
                except Exception as e:
                    self._log.debug("iter_measurements() failed: %s", e)

            if has_iter_measurments:
                try:
                    self._meas_iter = getattr(self._lidar, "iter_measurments")(max_buf_meas=self._max_buf_meas)
                    self._mode = "measurments"
                    self._scanning = True
                    self._log.info("RPLidar: using iter_measurments(max_buf_meas=%d)", self._max_buf_meas)
                    return True
                except Exception as e:
                    self._log.debug("iter_measurments() failed: %s", e)

            # iter_scans fallback: STANDARD → EXPRESS → bare
            if has_iter_scans:
                try:
                    self._scan_iter = getattr(self._lidar, "iter_scans")(
                        scan_type="standard", max_buf_meas=self._max_buf_meas, min_len=5
                    )
                    self._mode = "scans"
                    self._scanning = True
                    self._log.info(
                        "RPLidar: using iter_scans(scan_type=standard, max_buf_meas=%d, min_len=5)",
                        self._max_buf_meas
                    )
                    return True
                except TypeError:
                    try:
                        self._scan_iter = getattr(self._lidar, "iter_scans")()
                        self._mode = "scans"
                        self._scanning = True
                        self._log.info("RPLidar: using iter_scans() fallback (no kwargs)")
                        return True
                    except Exception as e:
                        self._log.debug("iter_scans() no-kwargs failed: %s", e)
                except Exception as e:
                    self._log.debug("iter_scans(standard, ...) failed: %s", e)

                # EXPRESS fallback
                try:
                    self._scan_iter = getattr(self._lidar, "iter_scans")(
                        scan_type="express", max_buf_meas=self._max_buf_meas, min_len=5
                    )
                    self._mode = "scans"
                    self._scanning = True
                    self._log.info(
                        "RPLidar: using iter_scans(scan_type=express, max_buf_meas=%d, min_len=5)",
                        self._max_buf_meas
                    )
                    return True
                except Exception as e:
                    self._log.debug("iter_scans(express, ...) failed: %s", e)

                # Bare iter_scans with kwargs
                try:
                    self._scan_iter = getattr(self._lidar, "iter_scans")(max_buf_meas=self._max_buf_meas)
                    self._mode = "scans"
                    self._scanning = True
                    self._log.info("RPLidar: using iter_scans() with max_buf_meas=%d", self._max_buf_meas)
                    return True
                except TypeError:
                    self._scan_iter = getattr(self._lidar, "iter_scans")()
                    self._mode = "scans"
                    self._scanning = True
                    self._log.info("RPLidar: using iter_scans() fallback (no kwargs)")
                    return True
                except Exception as e:
                    self._log.debug("iter_scans(max_buf_meas=..) failed: %s", e)

            self._log.error("No supported iterator available on RPLidar object")
            self._meas_iter = None
            self._scan_iter = None
            self._scanning = False
            return False

        except Exception as e:
            self._log.error("start_scan exception: %s", e)
            self._meas_iter = None
            self._scan_iter = None
            self._scanning = False
            return False

    def stop_scan(self) -> None:
        if self._lidar is not None:
            try:
                self._lidar.stop()
            except Exception:
                pass
            try:
                self._lidar.stop_motor()
            except Exception:
                pass
        self._scanning = False
        self._meas_iter = None
        self._scan_iter = None

    def get_scan_data(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        """
        Return one revolution as (angles_deg, ranges_mm, qualities) or None.
        measurement modes: consume until 'new_scan' toggles.
        scans mode: each list from iter_scans() is one revolution.
        """
        if not self._scanning:
            return None

        # measurement modes
        if self._mode in ("measurements", "measurments") and self._meas_iter is not None:
            q: List[int] = []
            a: List[float] = []
            d: List[float] = []
            started = False
            t0 = time.time()
            try:
                while True:
                    new_scan, quality, angle, dist = next(self._meas_iter)
                    if quality is None:
                        quality = 0
                    if new_scan and started:
                        break
                    started = True
                    q.append(int(quality))
                    a.append(float(angle))
                    d.append(float(dist))
                    if (time.time() - t0) > 0.25 and len(a) > 10:
                        break
            except StopIteration:
                return None
            except Exception:
                return None
            return (a, d, q)

        # scans mode
        if self._mode == "scans" and self._scan_iter is not None:
            try:
                scan = next(self._scan_iter)  # list of (quality, angle, distance)
                q: List[int] = []
                a: List[float] = []
                d: List[float] = []
                for quality, angle, dist in scan:
                    q.append(int(quality)); a.append(float(angle)); d.append(float(dist))
                return (a, d, q)
            except StopIteration:
                return None
            except Exception:
                return None

        return None


# -------------------- SDK bridge backend (launch ultra_simple) ----------------
class _SDKBridgeBackend:
    """
    Run Slamtec SDK 'ultra_simple' and parse stdout lines like:
      "   theta: 0.08 Dist: 05118.00 Q: 47"
    Handshake: mark scanning only after a parsable line arrives.
    """
    _LINE_RE = re.compile(
        r"^\s*(?:S\s+)?theta:\s*([0-9]+(?:\.[0-9]+)?)\s+"
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
        wrap_threshold_deg: float = 45.0,
        use_channel_args: bool = True,
        force_line_buffering: bool = True,
        use_pty: bool = True,
        prefer_pty: bool = True,
        max_revolution_ms: float = 250.0,
        debug_log_first_lines: int = 40,
        launch_timeout_sec: float = 5.0,
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

        self._proc: Optional[subprocess.Popen] = None
        self._stdout = None
        self._stderr = None
        self._pty_master_fd: Optional[int] = None
        self._pty_child: bool = False
        self._pty_buf: str = ""

        self._a: List[float] = []
        self._r: List[float] = []
        self._q: List[int] = []
        self._last_ang: Optional[float] = None
        self._rev_start_ts: float = 0.0

        self._connected = False
        self._scanning = False
        self._log = logging.getLogger(__name__)

    def connect(self) -> bool:
        if not os.path.isfile(self._path) or not os.access(self._path, os.X_OK):
            self._log.error("SDK ultra_simple not found or not executable: %s", self._path)
            self._connected = False
            return False
        self._connected = True
        return True

    def _base_args(self) -> List[str]:
        return ["--channel", "--serial", self._port, str(self._baud)] if self._use_channel else [self._port, str(self._baud)]

    def _cmd_stdbuf(self) -> List[str]:
        base = [self._path] + self._base_args()
        if self._force_linebuf:
            return ["stdbuf", "-oL", "-eL"] + base
        return base

    def _launch_stdbuf(self) -> bool:
        try:
            cmd = self._cmd_stdbuf()
            self._log.info("SDK ultra_simple launching (stdbuf): %s", " ".join(cmd))
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self._stdout = self._proc.stdout
            self._stderr = self._proc.stderr
            self._pty_child = False
            return True
        except Exception as e:
            self._log.error("stdbuf launch failed: %s", e)
            self._proc = None
            self._stdout = None
            self._stderr = None
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
            self._stdout = None
            self._stderr = None
            self._pty_child = True
            self._pty_buf = ""
            return True
        except Exception as e:
            self._log.error("PTY launch failed: %s", e)
            self._proc = None
            self._pty_master_fd = None
            return False

    def start_scan(self) -> bool:
        if not self._connected:
            return False

        # Prefer PTY immediately if configured
        if self._prefer_pty and self._use_pty:
            if not self._launch_pty():
                return False
            t0 = time.time()
            while (time.time() - t0) < self._launch_timeout_sec:
                line = self._read_line_once()
                if line and self._parse_line(line):
                    self._scanning = True
                    self._debug_echo(line)        # INFO echo
                    return True
                time.sleep(0.05)
            self._log.error("SDK bridge (PTY): no parsable output; check rotor power/SDK build")
            return False

        # stdbuf path first (pipes)
        if not self._launch_stdbuf():
            if self._use_pty and not self._launch_pty():
                return False

        t0 = time.time()
        while (time.time() - t0) < self._launch_timeout_sec:
            line = self._read_line_once()
            if line and self._parse_line(line):
                self._scanning = True
                self._debug_echo(line)            # INFO echo
                return True
            time.sleep(0.05)

        # Switch to PTY if stdbuf produced no parsable lines
        if not self._pty_child and self._use_pty:
            self._log.warning("No parsable output from stdbuf path; switching to PTY")
            self.stop_scan()
            if not self._launch_pty():
                return False
            t1 = time.time()
            while (time.time() - t1) < self._launch_timeout_sec:
                line = self._read_line_once()
                if line and self._parse_line(line):
                    self._scanning = True
                    self._debug_echo(line)        # INFO echo
                    return True
                time.sleep(0.05)

        self._log.error("SDK bridge: no parsable output; check rotor power/SDK build")
        return False

    def stop_scan(self) -> None:
        if self._proc:
            try:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=2.0)
                except Exception:
                    self._proc.kill()
            except Exception:
                pass

        if self._pty_master_fd is not None:
            try:
                os.close(self._pty_master_fd)
            except Exception:
                pass

        self._proc = None
        self._stdout = None
        self._stderr = None
        self._pty_master_fd = None
        self._scanning = False
        self._a.clear(); self._r.clear(); self._q.clear()
        self._last_ang = None
        self._rev_start_ts = 0.0
        self._pty_buf = ""

    def _clean_line(self, s: str) -> str:
        # Strip ANSI color codes and normalize CRLF; keep printable/text whitespace
        s2 = self.ANSI_RE.sub("", s)
        s2 = s2.replace("\r", "")
        return "".join(ch for ch in s2 if ch.isprintable() or ch in "\n\t ")

    def _parse_line(self, line: str) -> Optional[Tuple[float, float, int]]:
        m = self._LINE_RE.search(line)
        if not m:
            # Token-based fallback (more forgiving)
            m1 = re.search(r"theta:\s*([0-9]+(?:\.[0-9]+)?)", line, re.IGNORECASE)
            m2 = re.search(r"Dist:\s*([0-9]+(?:\.[0-9]+)?)",  line, re.IGNORECASE)
            m3 = re.search(r"Q:\s*(\d+)",                     line, re.IGNORECASE)
            if not (m1 and m2 and m3):
                return None
            try:
                ang = float(m1.group(1))
                dist = float(m2.group(1))
                qual = int(m3.group(1))
                return ang, dist, qual
            except Exception:
                return None

        try:
            ang = float(m.group(1))
            dist = float(m.group(2))
            qual = int(m.group(3))
            # Optional: log parsed triples at DEBUG
            self._log.debug("parsed triple: theta=%.2f dist=%.2f Q=%d", ang, dist, qual)
            return ang, dist, qual
        except Exception:
            return None

    def _is_wrap(self, ang: float) -> bool:
        return self._last_ang is not None and (ang + self._wrap) < self._last_ang

    def _maybe_publish_partial(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        if self._rev_start_ts == 0.0:
            return None
        if (time.time() - self._rev_start_ts) * 1000.0 >= self._max_rev_ms and len(self._a) >= 4:
            out = (self._a[:], self._r[:], self._q[:])
            self._a.clear(); self._r.clear(); self._q.clear()
            self._last_ang = None
            self._rev_start_ts = 0.0
            return out
        return None

    def _debug_echo(self, line: str, raw: bool = False):
        # Echo first N lines at INFO for guaranteed visibility
        if self._debug_lines_left > 0:
            if raw:
                self._log.info("ultra_simple[stdout/raw]: %r", line)
            else:
                self._log.info("ultra_simple[stdout]: %s", line.strip())
            self._debug_lines_left -= 1

    def _read_line_once(self) -> Optional[str]:
        """
        Read a single line from PTY or pipe.
        Echo raw repr and cleaned text (INFO) so ANSI/control codes are visible.
        """
        try:
            if self._pty_child and self._pty_master_fd is not None:
                rlist, _, _ = select.select([self._pty_master_fd], [], [], 0)
                if rlist:
                    data = os.read(self._pty_master_fd, 4096)
                    if not data:
                        return None
                    self._pty_buf += data.decode(errors="ignore")
                    if "\n" in self._pty_buf:
                        lines = self._pty_buf.splitlines()
                        if not self._pty_buf.endswith("\n"):
                            self._pty_buf = lines[-1]
                            lines = lines[:-1]
                        else:
                            self._pty_buf = ""
                        # Echo last few raw lines and cleaned text
                        for raw_ln in lines[-min(3, len(lines)):]:
                            self._debug_echo(raw_ln, raw=True)
                            cleaned = self._clean_line(raw_ln)
                            self._debug_echo(cleaned, raw=False)
                        # Return first scan-like line
                        for ln in lines:
                            cln = self._clean_line(ln)
                            if "theta:" in cln and "Dist:" in cln and "Q:" in cln:
                                return cln
                        # Return last banner line
                        if lines:
                            return self._clean_line(lines[-1])
                    return None
            else:
                if self._stdout:
                    ln = self._stdout.readline()
                    if ln:
                        self._debug_echo(ln, raw=True)
                        cleaned = self._clean_line(ln)
                        self._debug_echo(cleaned, raw=False)
                        if ("theta:" in cleaned and "Dist:" in cleaned and "Q:" in cleaned):
                            return cleaned
                        return cleaned or None
                return None
        except Exception as e:
            self._log.debug("read_line_once exception: %s", e)
            return None

    def get_scan_data(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        if not self._scanning:
            return None

        line = self._read_line_once()
        if not line:
            part = self._maybe_publish_partial()
            if part:
                return part
            return None

        parsed = self._parse_line(line)
        if not parsed:
            return None

        ang, dist_mm, qual = parsed
        if self._rev_start_ts == 0.0:
            self._rev_start_ts = time.time()

        # Angle wrap -> publish revolution
        if self._is_wrap(ang):
            if len(self._a) > 0:
                out = (self._a[:], self._r[:], self._q[:])
                self._a.clear(); self._r.clear(); self._q.clear()
                self._last_ang = None
                self._rev_start_ts = 0.0
                return out

        # Accumulate
        self._a.append(ang)
        self._r.append(dist_mm)
        self._q.append(qual)
        self._last_ang = ang

        part = self._maybe_publish_partial()
        if part:
            return part

        return None


# --------------------------------- Adapter -----------------------------------
class RPLidarS2Adapter(AbstractSensorAdapter):
    """RPLidar S2 adapter that supports Python driver or SDK bridge backend."""

    def __init__(
        self,
        sensor_id: str,
        kind: str = "lidar2d",
        port: Optional[str] = "/dev/ttyUSB0",
        baud: int = 115200,
        hz: Optional[float] = None,
        publish_empty_scans: bool = False,
        startup_delay: float = 1.0,
        poll_interval: float = 0.02,
        min_points: int = 120,
        empty_log_interval: float = 2.0,
        no_data_reconnect_sec: float = 8.0,
        backoff_initial: float = 1.0,
        backoff_max: float = 5.0,
        max_points: int = 4096,
        timeout: float = 2.5,
        use_sdk_bridge: bool = False,
        ultra_simple_path: str = "/home/dev/rplidar_sdk/output/Linux/Release/ultra_simple",
        wrap_threshold_deg: float = 45.0,
        use_channel_args: bool = True,
        force_line_buffering: bool = True,
        use_pty: bool = True,
        prefer_pty: bool = True,
        max_revolution_ms: float = 250.0,
        debug_log_first_lines: int = 40,
        max_buf_meas: int = 10000,
        drain_scans_per_tick: int = 5,
        fallback_to_python_on_sdk_failure: bool = True,
        # >>> NEW: allow config to specify longer bridge handshake timeout
        launch_timeout_sec: float = 5.0,
    ):
        super().__init__(sensor_id, kind)

        self.logger = getattr(
            self,
            "logger",
            logging.getLogger(f"sensorhub.adapters.rplidar_s2.{self.__class__.__name__}.{sensor_id}")
        )
        self._stop = getattr(self, "_stop", threading.Event())

        # Config
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

        # Backend selection
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
        self._sdk_launch_timeout_sec = float(launch_timeout_sec)  # <<< NEW

        # State
        self._driver = None
        self._last_empty_log = 0.0
        self._last_pub_ts: float = 0.0
        self._min_pub_period: float = (1.0 / hz) if (hz and hz > 0) else 0.0
        self._last_scan_ts: float = 0.0
        self._reconnecting: bool = False

        # Latest-frame cache for polling
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

        # Optional info/health (Python backend only)
        try:
            if isinstance(self._driver, _RplidarBackend):
                info = self._driver.get_device_info()
                self.logger.info("RPLIDAR S2 info=%s", info)
        except Exception:
            pass
        try:
            if isinstance(self._driver, _RplidarBackend):
                health = self._driver.get_health()
                self.logger.info("RPLIDAR S2 health=%s", health)
        except Exception:
            pass

        if not self._driver.start_scan():
            self.logger.error("start_scan failed (backend=%s)", backend_name)
            if self._use_sdk_bridge and self._fallback_to_python:
                self.logger.warning("SDK bridge failed to produce parsable output; falling back to Python driver.")
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
                        raise RuntimeError("Python backend start_scan failed after SDK bridge failure")
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
                try:
                    self._driver.stop_scan()
                except Exception:
                    pass
                try:
                    if hasattr(self._driver, "disconnect"):
                        self._driver.disconnect()  # type: ignore
                except Exception:
                    pass
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
                launch_timeout_sec=self._sdk_launch_timeout_sec,  # <<< NEW
            )
        return _RplidarBackend(self.port, self.baud, timeout=self.timeout, max_buf_meas=self._max_buf_meas)

    def _decimate(self, a: List[float], r: List[float], q: List[int]) -> Tuple[List[float], List[float], List[int]]:
        n = len(a)
        if n <= self._max_points:
            return a, r, q
        step = max(1, n // self._max_points)
        return a[::step], r[::step], q[::step]

    def _should_publish(self, now: float) -> bool:
        return self._min_pub_period == 0.0 or (now - self._last_pub_ts) >= self._min_pub_period

    def _schedule_reconnect_if_needed(self, now: float) -> None:
        if (
            self._no_data_reconnect_sec > 0
            and (now - self._last_scan_ts) >= self._no_data_reconnect_sec
            and not self._reconnecting
        ):
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
                    time.sleep(backoff)
                    backoff = min(backoff * 2, self._backoff_max)
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

                    if not scan:
                        break

                    a, r, q = scan
                    now = time.time()
                    self._last_scan_ts = now

                    # Update latest_raw cache
                    if len(a) >= 4:
                        a2, r2, q2 = self._decimate(a, r, q)
                        self._latest_frame = {
                            "sensor_id": self.sensor_id,
                            "angles": a2,
                            "ranges": r2,
                            "qualities": q2,
                            "timestamp": now,
                            "partial": len(a) < self._min_points
                        }

                    # Publish when gates satisfied
                    if len(a) >= self._min_points and self._should_publish(now):
                        a2, r2, q2 = self._decimate(a, r, q)
                        frame = {
                            "sensor_id": self.sensor_id,
                            "angles": a2,
                            "ranges": r2,
                            "qualities": q2,
                            "timestamp": now,
                        }
                        self.publish(frame)
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

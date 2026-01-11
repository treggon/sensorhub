
# src/sensorhub/adapters/rplidar_s2/rplidar_adapter.py
"""
RPLidar S2 adapter with full-revolution aggregation, robust SDK bridge parsing,
auto-probing of CLI arguments (fixes 'Usage' / no-data startup), optional Python
driver fallback, decimation/smoothing, and rich health metrics.

Designed for drop-in use with SensorHub:
- Accepts both 'baud' and 'baudrate' from YAML
- Publishes ONLY full 360° revolutions (or on timeout)
- Handles PTY or PIPE for the SDK bridge with safe teardown
- Auto-probes multiple CLI argument styles for 'ultra_simple'
- Watchdogs for no-data/no-publish
- Extensive configuration knobs and defensive logging

YAML example
------------
- id: rplidar1
  kind: lidar2d
  module: sensorhub.adapters.rplidar_s2.rplidar_adapter
  class: RPLidarS2Adapter
  params:
    port: /dev/ttyUSB0
    baud: 1000000
    require_wrap_for_publish: true
    wrap_threshold_deg: 1.0
    min_points: 180
    max_points: 4096
    max_revolution_ms: 1000.0
    drain_scans_per_tick: 16
    poll_interval: 0.002
    strict_parse_only: false
    enable_latest_raw_cache: true
    use_sdk_bridge: true
    ultra_simple_path: /home/dev/treggon/rplidar_sdk/output/Linux/Release/ultra_simple
    use_pty: true
    prefer_pty: true
    force_line_buffering: true
    use_channel_args: true
    launch_timeout_sec: 15.0
    fastpath_after_handshake: true
    debug_log_first_lines: 30
    echo_stdout: true
    log_parsed_triples: false
    fallback_to_python_on_sdk_failure: true

Optional params (defaults shown)
--------------------------------
  angle_units_deg: true
  normalize_angles: true
  drop_out_of_range: true
  min_quality: 0
  max_distance_mm: 0
  target_points_per_rev: 0
  smoothing_window: 0
  smoothing_strategy: "mean"
  duplicate_angle_resolution: 0.0
  watchdog_no_data_ms: 3000.0
  watchdog_no_publish_ms: 5000.0
  hard_fail_on_bridge_error: false
"""

from __future__ import annotations

import os
import re
import pty
import time
import shlex
import errno
import fcntl
import math
import select
import subprocess
import logging
from statistics import median
from typing import Iterable, List, Optional, Tuple, Dict, Any

from sensorhub.core.sensor_base import AbstractSensorAdapter


# =====================================================================================
# Logging
# =====================================================================================

log = logging.getLogger("sensorhub.adapters.rplidar_s2.RPLidarS2Adapter")


# =====================================================================================
# Parsing utilities
# =====================================================================================


STRICT_PATTERNS = [
    # 1) CSV, optional leading 'S': "S, angle, dist, qual" OR "angle, dist, qual"
    re.compile(r"^\s*(?P<s>S)?\s*,?\s*(?P<a>-?\d+(?:\.\d+)?)\s*,\s*(?P<d>\d+(?:\.\d+)?)\s*,\s*(?P<q>\d+)\s*$"),

    # 2) Keyed fields (optional 'start'): "angle=..., distance=..., quality=..., start=0|1"
    re.compile(
        r"^\s*angle\s*[:=]\s*(?P<a>-?\d+(?:\.\d+)?)\s*,\s*distance\s*[:=]\s*(?P<d>\d+(?:\.\d+)?)\s*,\s*quality\s*[:=]\s*(?P<q>\d+)(?:\s*,\s*start\s*[:=]\s*(?P<start>[01]))?\s*$",
        re.IGNORECASE,
    ),

    # 3) Tokenized (optional leading 'S'): "S angle 123.45 dist 678.0 qual 15"
    re.compile(
        r"^\s*(?P<s>S)?\s*angle\s+(?P<a>-?\d+(?:\.\d+)?)\s+dist\s+(?P<d>\d+(?:\.\d+)?)\s+qual\s+(?P<q>\d+)\s*$",
        re.IGNORECASE,
    ),

    # 4) Ultra_simple style (your SDK): "theta: 0.10 Dist: 00153.00 Q: 47"
    re.compile(
        r"^\s*theta\s*[:=]\s*(?P<a>-?\d+(?:\.\d+)?)\s+Dist\s*[:=]\s*(?P<d>\d+(?:\.\d+)?)\s+Q\s*[:=]\s*(?P<q>\d+)\s*$",
        re.IGNORECASE,
    ),
]



def _try_parse_fast_csv(line: str) -> Optional[Dict[str, Any]]:
    s = line.strip()
    if not s:
        return None
    parts = [p.strip() for p in s.split(",")]
    if 3 <= len(parts) <= 4:
        try:
            idx = 0
            is_start = False
            first = parts[0].upper()
            if first == "S":
                is_start = True
                idx = 1
            angle = float(parts[idx])
            dist = float(parts[idx + 1])
            qual = int(float(parts[idx + 2]))
            return {"angle": angle, "distance": dist, "quality": qual, "is_start": is_start}
        except Exception:
            return None
    return None


def parse_line_to_triple(line: str, strict: bool = False) -> Optional[Dict[str, Any]]:
    if not strict:
        t = _try_parse_fast_csv(line)
        if t is not None:
            return t
    s = line.strip()
    if not s:
        return None
    for pat in STRICT_PATTERNS:
        m = pat.match(s)
        if not m:
            continue
        try:
            angle = float(m.group("a"))
            dist = float(m.group("d"))
            qual = int(m.group("q"))
            is_start = False
            if m.groupdict().get("s"):
                is_start = True
            start_num = m.groupdict().get("start")
            if start_num is not None:
                try:
                    is_start = bool(int(start_num))
                except Exception:
                    pass
            return {"angle": angle, "distance": dist, "quality": qual, "is_start": is_start}
        except Exception:
            continue
    return None


# =====================================================================================
# Helper classes: Decimation & Smoothing
# =====================================================================================

class AngleDecimator:
    def __init__(self, target_points: int = 0) -> None:
        self.target = max(0, int(target_points))

    def apply(self, points: List[Tuple[float, float, int]]) -> List[Tuple[float, float, int]]:
        n = len(points)
        if self.target <= 0 or self.target >= n or n == 0:
            return points
        step = n / float(self.target)
        out: List[Tuple[float, float, int]] = []
        i = 0.0
        while int(i) < n and len(out) < self.target:
            out.append(points[int(i)])
            i += step
        return out


class AngleSmoother:
    def __init__(self, window: int = 0, strategy: str = "mean") -> None:
        self.window = max(0, int(window))
        self.strategy = strategy.lower().strip()

    def apply(self, points: List[Tuple[float, float, int]]) -> List[Tuple[float, float, int]]:
        if self.window <= 0 or len(points) <= 2 or self.strategy not in ("mean", "median"):
            return points
        win = self.window
        half = win // 2
        n = len(points)
        out: List[Tuple[float, float, int]] = []
        for i in range(n):
            lo = max(0, i - half)
            hi = min(n, i + half + 1)
            window_pts = points[lo:hi]
            if not window_pts:
                out.append(points[i])
                continue
            distances = [d for _, d, _ in window_pts]
            smooth_dist = sum(distances) / float(len(distances)) if self.strategy == "mean" else float(median(distances))
            angle = points[i][0]
            quality = points[i][2]
            out.append((angle, smooth_dist, quality))
        return out


# =====================================================================================
# RPLidarS2Adapter
# =====================================================================================

class RPLidarS2Adapter(AbstractSensorAdapter):
    """
    Reads scan triples from SDK bridge (preferred) or Python driver fallback,
    aggregates points into full revolutions, applies optional decimation/smoothing,
    and publishes structured payloads.
    """

    def __init__(self, sensor_id: str, kind: str, **kwargs) -> None:
        super().__init__(sensor_id=sensor_id, kind=kind)
        self._log = logging.getLogger(f"sensorhub.adapters.rplidar_s2.RPLidarS2Adapter.{sensor_id}")

        self.port: str = kwargs.get("port", "/dev/ttyUSB0")
        self.baud: int = int(kwargs.pop("baud", kwargs.pop("baudrate", 115200)))

        for k, v in kwargs.items():
            setattr(self, k, v)

        self._proc: Optional[subprocess.Popen] = None
        self._pty_master_fd: Optional[int] = None
        self._pty_slave_fd: Optional[int] = None

        # Python fallback state
        self._py_driver = None
        self._py_iter = None
        self._py_fallback_enabled: bool = False

        # Metrics
        self._revolutions_published: int = 0
        self._last_publish_count: int = 0
        self._parse_errors: int = 0
        self._bridge_lines_seen: int = 0
        self._triples_parsed: int = 0
        self._rollovers_detected: int = 0
        self._start_flags_detected: int = 0
        self._last_data_ts_monotonic: float = time.monotonic()
        self._last_publish_ts_monotonic: float = time.monotonic()
        self._watchdog_warned_no_data: bool = False
        self._watchdog_warned_no_publish: bool = False

        self._latest_raw_points: Optional[List[Dict[str, Any]]] = None

        if getattr(self, "description", None) is None:
            self.description = f"RPLidar S2 on {self.port}@{self.baud}"

        self._apply_default_tunables()

    def _apply_default_tunables(self) -> None:
        defaults: Dict[str, Any] = {
            "angle_units_deg": True,
            "normalize_angles": True,
            "drop_out_of_range": True,
            "min_quality": 0,
            "max_distance_mm": 0.0,
            "target_points_per_rev": 0,
            "smoothing_window": 0,
            "smoothing_strategy": "mean",
            "duplicate_angle_resolution": 0.0,
            "watchdog_no_data_ms": 3000.0,
            "watchdog_no_publish_ms": 5000.0,
            "hard_fail_on_bridge_error": False,
        }
        for k, v in defaults.items():
            if not hasattr(self, k):
                setattr(self, k, v)

    # -----------------------------------------------------------------------------
    # Python fallback
    # -----------------------------------------------------------------------------
    def _try_python_fallback(self) -> bool:
        allow_fallback = bool(getattr(self, "fallback_to_python_on_sdk_failure", True))
        if not allow_fallback:
            return False
        try:
            import rplidar  # type: ignore
        except Exception as e:
            self._log.info("Python fallback not available (rplidar import failed): %s", e)
            return False
        try:
            self._py_driver = rplidar.RPLidar(self.port, baudrate=self.baud)
            max_buf = int(getattr(self, "max_buf_meas", 10000))
            self._py_iter = self._py_driver.iter_measurements(max_buf)
            self._py_fallback_enabled = True
            self._log.info("Using Python rplidar fallback on %s@%d", self.port, self.baud)
            return True
        except Exception as e:
            self._log.error("Python fallback init failed: %s", e)
            self._py_driver = None
            self._py_iter = None
            self._py_fallback_enabled = False
            return False

    def _python_fallback_read_triples(self, max_count: int = 1024) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        if not self._py_iter:
            return out
        for _ in range(max_count):
            try:
                meas = next(self._py_iter)
            except StopIteration:
                break
            except Exception:
                break
            if not isinstance(meas, (tuple, list)) or len(meas) < 3:
                continue
            qual, angle, dist = int(meas[0]), float(meas[1]), float(meas[2])
            out.append({"angle": angle, "distance": dist, "quality": qual, "is_start": False})
        self._last_data_ts_monotonic = time.monotonic()
        return out

    def _python_fallback_close(self) -> None:
        try:
            if self._py_driver:
                try:
                    self._py_driver.stop()
                except Exception:
                    pass
                try:
                    self._py_driver.disconnect()
                except Exception:
                    pass
        finally:
            self._py_driver = None
            self._py_iter = None
            self._py_fallback_enabled = False

    # -----------------------------------------------------------------------------
    # SDK bridge: auto-probe CLI args
    # -----------------------------------------------------------------------------
    def _candidate_bridge_cmds(self) -> List[List[str]]:
        """
        Build a list of candidate command lines for 'ultra_simple' to handle
        the different argument styles across SDK versions.
        """
        path = str(getattr(self, "ultra_simple_path", "")).strip()
        if not path:
            return []

        cands: List[List[str]] = []

        # Preferred per SDK usage output:
        #   --channel --serial <com port> [baudrate]
        cands.append([path, "--channel", "--serial", self.port, str(self.baud)])
        cands.append([path, "--channel", "--serial", self.port])  # baud inferred by model

        # Legacy/other variations (keep as fallback)
        cands.append([path, "--port", self.port, "--baud", str(self.baud)])
        cands.append([path, "-p", self.port, "-b", str(self.baud)])

        # Wrap with stdbuf if requested
        force_lb = bool(getattr(self, "force_line_buffering", True))
        if force_lb:
            cands = [["stdbuf", "-oL", "-eL"] + cmd for cmd in cands]

        return cands

    def _launch_one_bridge(self, cmd: List[str], use_pty: bool, prefer_pty: bool) -> bool:
        """
        Launch a single candidate command. Returns True if the process starts
        and does NOT immediately print 'Usage' lines (i.e., likely good).
        """
        try:
            if use_pty and prefer_pty:
                master_fd, slave_fd = pty.openpty()
                self._pty_master_fd = master_fd
                self._pty_slave_fd = slave_fd
                self._proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=slave_fd,
                    stderr=slave_fd,
                    text=True,
                    bufsize=1,
                    close_fds=True,
                )
                flags = fcntl.fcntl(master_fd, fcntl.F_GETFL)
                fcntl.fcntl(master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            else:
                self._proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    close_fds=True,
                )

            self._log.info("Attempting SDK bridge: %s", " ".join(shlex.quote(c) for c in cmd))

            # Probe initial output up to ~0.8s to detect 'Usage' banner
            t0 = time.monotonic()
            bad = False
            while (time.monotonic() - t0) < 0.8:
                lines = self._bridge_read_lines(max_lines=256)
                if lines:
                    for ln in lines:
                        # Echo a few lines for visibility
                        self._log.debug("[SDK] %s", ln)
                        if "Usage" in ln or "For serial channel" in ln:
                            bad = True
                            break
                if bad:
                    break
                # If process exited immediately, treat as bad
                if self._proc and self._proc.poll() is not None:
                    bad = True
                    break
                time.sleep(0.05)

            if bad:
                self._log.warning("Bridge candidate printed usage/failed; will try next.")
                self._terminate_bridge()
                return False

            self._log.info("Bridge candidate looks good.")
            return True

        except Exception as e:
            self._log.error("Bridge candidate launch failed: %s", e)
            self._terminate_bridge()
            return False

    def _launch_bridge(self) -> bool:
        """
        Try multiple candidate commands until one works. Returns True if a viable
        bridge is running; False otherwise.
        """
        use_bridge = bool(getattr(self, "use_sdk_bridge", True))
        if not use_bridge:
            self._log.warning("use_sdk_bridge=False; will attempt Python fallback if allowed.")
            return False

        path = str(getattr(self, "ultra_simple_path", "")).strip()
        if not path or not os.path.exists(path):
            self._log.error("ultra_simple_path not found: %s", path)
            self.last_error = f"ultra_simple_path not found: {path}"
            return False

        cands = self._candidate_bridge_cmds()
        if not cands:
            self._log.error("No bridge command candidates built.")
            return False

        use_pty = bool(getattr(self, "use_pty", True))
        prefer_pty = bool(getattr(self, "prefer_pty", True))

        for cmd in cands:
            if self._launch_one_bridge(cmd, use_pty, prefer_pty):
                return True

        # All candidates failed
        self.last_error = "All bridge command candidates failed"
        return False

    def _bridge_read_lines(self, max_lines: int = 1024) -> List[str]:
        out_lines: List[str] = []
        if self._pty_master_fd is not None:
            fd = self._pty_master_fd
            rlist, _, _ = select.select([fd], [], [], 0.0)
            if fd in rlist:
                try:
                    data = os.read(fd, 8192).decode(errors="ignore")
                    if data:
                        lines = data.splitlines()
                        out_lines.extend(lines)
                        self._bridge_lines_seen += len(lines)
                except OSError as e:
                    if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                        self._log.debug("PTY read error: %s", e)
            if out_lines:
                self._last_data_ts_monotonic = time.monotonic()
            return out_lines

        if self._proc and self._proc.stdout:
            for _ in range(max_lines):
                rlist, _, _ = select.select([self._proc.stdout], [], [], 0.0)
                if self._proc.stdout in rlist:
                    line = self._proc.stdout.readline()
                    if not line:
                        break
                    out_lines.append(line.rstrip("\n"))
                    self._bridge_lines_seen += 1
                else:
                    break

        if out_lines:
            self._last_data_ts_monotonic = time.monotonic()
        return out_lines

    def _terminate_bridge(self) -> None:
        try:
            if self._proc:
                if self._proc.poll() is None:
                    try:
                        self._proc.terminate()
                    except Exception:
                        pass
                    try:
                        self._proc.wait(timeout=1.5)
                    except Exception:
                        try:
                            self._proc.kill()
                        except Exception:
                            pass
                self._proc = None
        finally:
            if self._pty_master_fd is not None:
                try:
                    os.close(self._pty_master_fd)
                except Exception:
                    pass
                self._pty_master_fd = None
            if self._pty_slave_fd is not None:
                try:
                    os.close(self._pty_slave_fd)
                except Exception:
                    pass
                self._pty_slave_fd = None

    # -----------------------------------------------------------------------------
    # Drain abstraction
    # -----------------------------------------------------------------------------
    def _drain_scans(self, count_hint: int = 16, strict: bool = False) -> List[Dict[str, Any]]:
        if self._proc or self._pty_master_fd is not None:
            lines = self._bridge_read_lines(max_lines=max(256, count_hint * 4))
            out: List[Dict[str, Any]] = []
            for ln in lines:
                t = parse_line_to_triple(ln, strict=strict)
                if t is None:
                    self._parse_errors += 1
                    continue
                out.append(t)
            self._triples_parsed += len(out)
            return out[:count_hint] if count_hint > 0 else out

        if self._py_fallback_enabled and self._py_iter is not None:
            out = self._python_fallback_read_triples(max_count=max(256, count_hint * 8))
            self._triples_parsed += len(out)
            return out[:count_hint] if count_hint > 0 else out

        return []

    # -----------------------------------------------------------------------------
    # Filtering / normalization
    # -----------------------------------------------------------------------------
    def _normalize_angle(self, angle: float, is_deg: bool, normalize: bool) -> float:
        a = math.degrees(angle) if not is_deg else angle
        if normalize:
            a = a % 360.0
            if a < 0.0:
                a += 360.0
        return a

    def _point_passes_filters(self, dist_mm: float, quality: int) -> bool:
        if bool(getattr(self, "drop_out_of_range", True)):
            if not math.isfinite(dist_mm) or dist_mm <= 0.0:
                return False
        min_q = int(getattr(self, "min_quality", 0))
        if quality < min_q:
            return False
        max_d = float(getattr(self, "max_distance_mm", 0.0))
        if max_d > 0.0 and dist_mm > max_d:
            return False
        return True

    def _collapse_duplicate_angles(
        self,
        points: List[Tuple[float, float, int]],
        resolution_deg: float
    ) -> List[Tuple[float, float, int]]:
        res = float(resolution_deg)
        if res <= 0.0 or not points:
            return points
        buckets: Dict[int, Tuple[float, float, int, int]] = {}
        for (a, d, q) in points:
            idx = int(a // res)
            entry = buckets.get(idx)
            if entry is None:
                buckets[idx] = (a, d, q, 1)
            else:
                sa, sd, _, c = entry
                buckets[idx] = (sa + a, sd + d, q, c + 1)
        out: List[Tuple[float, float, int]] = []
        for idx, (sa, sd, q, c) in buckets.items():
            out.append((sa / float(c), sd / float(c), q))
        out.sort(key=lambda t: t[0])
        return out

    # -----------------------------------------------------------------------------
    # Main run loop
    # -----------------------------------------------------------------------------
    def run(self) -> None:
        wrap_thresh_deg = float(getattr(self, "wrap_threshold_deg", 1.0))
        require_wrap = bool(getattr(self, "require_wrap_for_publish", True))
        min_points = int(getattr(self, "min_points", 180))
        max_points = int(getattr(self, "max_points", 4096))
        max_rev_ms = float(getattr(self, "max_revolution_ms", 1000.0))
        drain_n = int(getattr(self, "drain_scans_per_tick", 16))
        poll_interval = float(getattr(self, "poll_interval", 0.002))
        strict_parse_only = bool(getattr(self, "strict_parse_only", False))
        echo_stdout = bool(getattr(self, "echo_stdout", False))
        debug_log_first = int(getattr(self, "debug_log_first_lines", 0))
        log_parsed_triples = bool(getattr(self, "log_parsed_triples", False))
        launch_timeout_sec = float(getattr(self, "launch_timeout_sec", 15.0))
        fastpath_after_handshake = bool(getattr(self, "fastpath_after_handshake", True))
        enable_latest_raw_cache = bool(getattr(self, "enable_latest_raw_cache", True))
        angle_units_deg = bool(getattr(self, "angle_units_deg", True))
        normalize_angles = bool(getattr(self, "normalize_angles", True))
        duplicate_angle_res = float(getattr(self, "duplicate_angle_resolution", 0.0))
        target_points = int(getattr(self, "target_points_per_rev", 0))
        smoothing_window = int(getattr(self, "smoothing_window", 0))
        smoothing_strategy = str(getattr(self, "smoothing_strategy", "mean"))

        decimator = AngleDecimator(target_points)
        smoother = AngleSmoother(smoothing_window, smoothing_strategy)

        launched = self._launch_bridge()
        if not launched:
            if not self._try_python_fallback():
                self._log.error("No scan source available (bridge failed and fallback not usable).")
                self.last_error = "No scan source"
                if bool(getattr(self, "hard_fail_on_bridge_error", False)):
                    return
            else:
                self._log.info("Proceeding with Python fallback.")

        # Warmup / handshake (limited echo)
        t0 = time.monotonic()
        echoed = 0
        while not self._stop.is_set():
            if self._proc or self._pty_master_fd is not None:
                lines = self._bridge_read_lines(max_lines=256)
                if lines and echo_stdout and echoed < debug_log_first:
                    for ln in lines:
                        if echoed >= debug_log_first:
                            break
                        self._log.debug("[SDK] %s", ln)
                        echoed += 1
                if fastpath_after_handshake and echoed >= debug_log_first:
                    break
                if (time.monotonic() - t0) > launch_timeout_sec:
                    self._log.info("Bridge launch timeout elapsed; proceeding to run loop.")
                    break
                time.sleep(0.01)
            else:
                break

        # Revolution accumulator
        rev_points: List[Tuple[float, float, int]] = []
        last_angle: Optional[float] = None
        last_wrap_t = time.monotonic()

        def publish_rev() -> None:
            if not rev_points:
                return
            rev_points.sort(key=lambda p: p[0])

            points = list(rev_points)
            if duplicate_angle_res > 0.0:
                points = self._collapse_duplicate_angles(points, duplicate_angle_res)
            if smoothing_window > 0:
                points = smoother.apply(points)
            points = decimator.apply(points)

            payload = [
                {"angle_deg": float(a), "distance_mm": float(d), "quality": int(q)}
                for (a, d, q) in points[:max_points]
            ]

            if enable_latest_raw_cache:
                try:
                    self._latest_raw_points = list(payload)
                except Exception:
                    self._latest_raw_points = None

            self._log.debug("rplidar1: raw scan points=%d (publish min=%d)", len(payload), min_points)

            if len(payload) >= min_points:
                self._last_publish_count = len(payload)
                self._revolutions_published += 1
                self._last_publish_ts_monotonic = time.monotonic()
                self.publish({"points": payload})

            rev_points.clear()

        try:
            while not self._stop.is_set():
                triples = self._drain_scans(count_hint=max(1, drain_n), strict=strict_parse_only)

                if log_parsed_triples and triples:
                    for t in triples:
                        self._log.debug(
                            "[parsed] angle=%.3f dist=%.3f qual=%d start=%s",
                            t["angle"], t["distance"], t["quality"], t.get("is_start", False)
                        )

                for t in triples:
                    a = self._normalize_angle(float(t["angle"]), angle_units_deg, normalize_angles)
                    d = float(t["distance"])
                    q = int(t.get("quality", 0))
                    is_start = bool(t.get("is_start", False))

                    if not self._point_passes_filters(d, q):
                        continue

                    rollover = (last_angle is not None and a < (last_angle - wrap_thresh_deg))
                    if rollover:
                        self._rollovers_detected += 1
                    if is_start:
                        self._start_flags_detected += 1

                    start = is_start or rollover
                    if start and require_wrap:
                        publish_rev()
                        last_wrap_t = time.monotonic()

                    rev_points.append((a, d, q))
                    last_angle = a

                if (time.monotonic() - last_wrap_t) * 1000.0 > max_rev_ms:
                    publish_rev()
                    last_wrap_t = time.monotonic()

                self._run_watchdogs()
                time.sleep(poll_interval)

        except Exception as e:
            self.last_error = f"{e.__class__.__name__}: {e}"
            self._log.error("RPLidar adapter run() error: %s", e)
        finally:
            self._terminate_bridge()
            self._python_fallback_close()

    # -----------------------------------------------------------------------------
    # Watchdogs
    # -----------------------------------------------------------------------------
    def _run_watchdogs(self) -> None:
        now = time.monotonic()
        no_data_ms = float(getattr(self, "watchdog_no_data_ms", 3000.0))
        no_pub_ms = float(getattr(self, "watchdog_no_publish_ms", 5000.0))

        if no_data_ms > 0:
            if (now - self._last_data_ts_monotonic) * 1000.0 > no_data_ms:
                if not self._watchdog_warned_no_data:
                    self._log.warning(
                        "RPLidar watchdog: no new data from bridge/fallback for %.0f ms",
                        no_data_ms
                    )
                    self._watchdog_warned_no_data = True
            else:
                self._watchdog_warned_no_data = False

        if no_pub_ms > 0:
            if (now - self._last_publish_ts_monotonic) * 1000.0 > no_pub_ms:
                if not self._watchdog_warned_no_publish:
                    self._log.warning(
                        "RPLidar watchdog: no publish for %.0f ms (points may be filtered below min=%d)",
                        no_pub_ms, int(getattr(self, "min_points", 180))
                    )
                    self._watchdog_warned_no_publish = True
            else:
                self._watchdog_warned_no_publish = False

    # -----------------------------------------------------------------------------
    # Health
    # -----------------------------------------------------------------------------
    def health(self) -> dict:
        base = super().health()
        try:
            with self._lock:
                base.update({
                    "description": getattr(self, "description", None),
                    "bridge_lines_seen": self._bridge_lines_seen,
                    "triples_parsed": self._triples_parsed,
                    "parse_errors": self._parse_errors,
                    "revolutions_published": self._revolutions_published,
                    "last_publish_count": self._last_publish_count,
                    "start_flags_detected": self._start_flags_detected,
                    "rollovers_detected": self._rollovers_detected,
                    "port": self.port,
                    "baud": self.baud,
                    "use_sdk_bridge": bool(getattr(self, "use_sdk_bridge", True)),
                    "python_fallback": bool(self._py_fallback_enabled),
                    "duplicate_angle_resolution": float(getattr(self, "duplicate_angle_resolution", 0.0)),
                    "target_points_per_rev": int(getattr(self, "target_points_per_rev", 0)),
                    "smoothing_window": int(getattr(self, "smoothing_window", 0)),
                    "smoothing_strategy": str(getattr(self, "smoothing_strategy", "mean")),
                    "min_quality": int(getattr(self, "min_quality", 0)),
                    "max_distance_mm": float(getattr(self, "max_distance_mm", 0.0)),
                    "drop_out_of_range": bool(getattr(self, "drop_out_of_range", True)),
                    "normalize_angles": bool(getattr(self, "normalize_angles", True)),
                    "angle_units_deg": bool(getattr(self, "angle_units_deg", True)),
                    "watchdog_no_data_ms": float(getattr(self, "watchdog_no_data_ms", 3000.0)),
                    "watchdog_no_publish_ms": float(getattr(self, "watchdog_no_publish_ms", 5000.0)),
                })
        except Exception:
            pass
        return base

    # -----------------------------------------------------------------------------
    # Utilities
    # -----------------------------------------------------------------------------
    def latest_raw_points(self) -> Optional[List[Dict[str, Any]]]:
        return self._latest_raw_points


"""
RPLidar S2 Adapter with SDK-or-rplidar fallback

- If pyrplidarsdk (Slamtec SDK Python wrapper) is available, use it.
- Otherwise, fall back to the pure-Python 'rplidar' library (serial only).

Patched backend:
- Starts scanning via iter_scans() (no redundant start_motor calls);
- Clears input buffers before scanning;
- Stops cleanly with stop() + stop_motor();
- Passes a quiet logger to suppress module-level "Starting motor" noise.
"""

import logging
import time
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple

from sensorhub.core.sensor_base import AbstractSensorAdapter


# ------------------------------ SDK backend -------------------------------

@dataclass
class _DeviceInfo:
    model: int
    firmware_version: int
    hardware_version: int
    serial_number: str


@dataclass
class _DeviceHealth:
    status: str  # "Good", "Warning", or "Error"
    error_code: int = 0


class _SDKBackend:
    """
    Thin wrapper around pyrplidarsdk to normalize methods used by the adapter.
    """

    def __init__(self, port: Optional[str], baudrate: int, ip: Optional[str], udp_port: int):
        import pyrplidarsdk  # type: ignore  # real SDK (Python >=3.10)

        if ip:
            self._driver = pyrplidarsdk.RplidarDriver(ip_address=ip, udp_port=udp_port)
        else:
            self._driver = pyrplidarsdk.RplidarDriver(port=port, baudrate=baudrate)

    def connect(self) -> bool:
        return bool(self._driver.connect())

    def disconnect(self) -> None:
        try:
            self._driver.disconnect()
        except Exception:
            pass

    def get_device_info(self) -> Optional[_DeviceInfo]:
        try:
            info = self._driver.get_device_info()
            if not info:
                return None
            return _DeviceInfo(
                model=int(getattr(info, "model", 0)),
                firmware_version=int(getattr(info, "firmware_version", 0)),
                hardware_version=int(getattr(info, "hardware_version", 0)),
                serial_number=str(getattr(info, "serial_number", "")),
            )
        except Exception:
            return None

    def get_health(self) -> Optional[_DeviceHealth]:
        try:
            health = self._driver.get_health()
            if not health:
                return None
            return _DeviceHealth(
                status=str(getattr(health, "status", "Unknown")),
                error_code=int(getattr(health, "error_code", 0)),
            )
        except Exception:
            return None

    def start_scan(self) -> bool:
        return bool(self._driver.start_scan())

    def stop_scan(self) -> None:
        try:
            self._driver.stop_scan()
        except Exception:
            pass

    def get_scan_data(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        """
        Return (angles_deg, ranges_mm, qualities) for one scan, or None.
        """
        try:
            data = self._driver.get_scan_data()
            if not data:
                return None
            angles, ranges, qualities = data
            return list(angles), list(ranges), list(qualities)
        except Exception:
            return None


# ---------------------------- rplidar backend -----------------------------

class _RplidarBackend:
    """
    Compatibility driver on top of the 'rplidar' library (serial only).
    Emulates the subset of the pyrplidarsdk API that this adapter uses.

    rplidar methods used: iter_scans(), clear_input(), stop(), stop_motor(),
    get_info(), get_health(), disconnect().

    A quiet logger is passed to the driver to suppress INFO messages like "Starting motor".
    """

    def __init__(self, port: Optional[str], baudrate: int, ip: Optional[str], udp_port: int, timeout: float = 1.0):
        if ip:
            # UDP transport not supported in rplidar fallback
            raise RuntimeError(
                "UDP transport (ip/udp_port) requires pyrplidarsdk; "
                "fallback 'rplidar' only supports serial."
            )
        try:
            from rplidar import RPLidar  # type: ignore
        except ImportError as e:
            raise ImportError("Fallback requires 'rplidar'. Install it: pip install rplidar") from e

        self._port = port or "/dev/ttyUSB0"
        self._baudrate = baudrate
        self._timeout = timeout
        self._RPLidar = RPLidar

        self._lidar: Optional[RPLidar] = None  # type: ignore
        self._scan_iter = None
        self._connected = False
        self._scanning = False

        # Optional kwargs for iter_scans() (leave empty for broad compatibility)
        self._scan_kwargs = {}

    def connect(self) -> bool:
        try:
            # Create a quiet logger for the rplidar module
            import logging as _logging
            quiet_logger = _logging.getLogger(f"sensorhub.rplidar.quiet.{self._port}")
            quiet_logger.propagate = False
            if not quiet_logger.handlers:
                h = _logging.StreamHandler()
                h.setFormatter(_logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s"))
                quiet_logger.addHandler(h)
            quiet_logger.setLevel(_logging.WARNING)  # suppress INFO like "Starting motor"

            # Pass the quiet logger to the driver
            self._lidar = self._RPLidar(
                self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
                logger=quiet_logger,
            )

            # Clear any stale bytes before scanning.
            try:
                self._lidar.clear_input()
            except Exception:
                pass

            # Optional: soft recovery if device reports ERROR.
            try:
                status, error_code = self._lidar.get_health()
                if str(status).lower() == "error":
                    try:
                        self._lidar.reset()
                        self._lidar.clear_input()
                    except Exception:
                        pass
            except Exception:
                # Some units may not return health reliably; proceed anyway.
                pass

            self._connected = True
            return True
        except Exception:
            self._lidar = None
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._lidar is not None:
            try:
                self.stop_scan()  # ensure measurement stream & motor are stopped
            except Exception:
                pass
            try:
                self._lidar.disconnect()
            except Exception:
                pass
            finally:
                self._lidar = None
        self._connected = False

    def get_device_info(self) -> Optional[_DeviceInfo]:
        if not self._connected or self._lidar is None:
            return None
        try:
            info = self._lidar.get_info()  # dict: model/firmware/hardware/serialnumber
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
            status, error_code = self._lidar.get_health()  # ('Good'|'Warning'|'Error', int)
            return _DeviceHealth(status=status, error_code=int(error_code))
        except Exception:
            return None

    def start_scan(self) -> bool:
        """
        Begin scanning via iter_scans(). Do NOT call start_motor() here—iter_scans()
        manages measurement streaming internally in the Python driver.
        """
        if not self._connected or self._lidar is None:
            return False
        try:
            # Clear any residual bytes before starting the generator.
            try:
                self._lidar.clear_input()
            except Exception:
                pass

            # Start the per‑revolution generator; this will stream scan batches.
            self._scan_iter = self._lidar.iter_scans(**self._scan_kwargs)
            self._scanning = True
            return True
        except Exception:
            self._scan_iter = None
            self._scanning = False
            return False

    def stop_scan(self) -> None:
        """
        Stop measurement stream and the motor for a clean shutdown.
        """
        if self._lidar is not None and self._scanning:
            try:
                self._lidar.stop()        # stop measurement stream
            except Exception:
                pass
            try:
                self._lidar.stop_motor()  # stop rotor
            except Exception:
                pass
            self._scanning = False
        self._scan_iter = None

    def get_scan_data(self) -> Optional[Tuple[List[float], List[float], List[int]]]:
        """
        Fetch exactly one revolution from iter_scans():
        returns (angles_deg, ranges_mm, qualities) or None.
        """
        if not self._scanning or self._scan_iter is None:
            return None
        try:
            scan = next(self._scan_iter)  # list of (quality:int, angle_deg:float, distance_mm:float)
            qualities: List[int] = []
            angles: List[float] = []
            distances: List[float] = []
            for q, ang, dist in scan:
                qualities.append(int(q))
                angles.append(float(ang))
                distances.append(float(dist))
            return (angles, distances, qualities)
        except StopIteration:
            return None
        except Exception:
            return None


# ----------------------------- Adapter class ------------------------------

class RPLidarS2Adapter(AbstractSensorAdapter):
    """
    RPLidar S2 adapter that prefers pyrplidarsdk, falls back to rplidar.
    """

    def __init__(
        self,
        sensor_id: str,
        kind: str = "lidar2d",
        port: Optional[str] = "/dev/ttyUSB0",
        baud: int = 1_000_000,
        ip: Optional[str] = None,
        udp_port: int = 8089,
        hz: Optional[float] = None,
        publish_empty_scans: bool = False,
    ):
        super().__init__(sensor_id, kind)
        self.logger = getattr(
            self,
            "logger",
            logging.getLogger(f"sensorhub.adapters.rplidar_s2.{self.__class__.__name__}.{sensor_id}")
        )
        self._stop = getattr(self, "_stop", threading.Event())

        self.port = port
        self.baud = baud
        self.ip = ip
        self.udp_port = udp_port
        self.hz = hz
        self.publish_empty_scans = publish_empty_scans

        # Tuning knobs
        self._startup_delay = 0.5          # seconds to let sampling spin up
        self._poll_interval = 0.02         # seconds between polls (≈50 Hz)
        self._min_points = 120             # lowered for quicker validation
        self._empty_log_interval = 2.0     # log "no data" at most once every 2s
        self._last_empty_log = 0.0

        self._driver = None
        self._thread: Optional[threading.Thread] = None
        self._backoff_sec = 1.0
        self._last_pub_ts: float = 0.0
        self._min_pub_period: float = (1.0 / hz) if (hz and hz > 0) else 0.0

        self._using_sdk = False  # True if pyrplidarsdk is in use

    def _make_backend(self):
        """
        Choose backend: pyrplidarsdk if importable, else rplidar.
        """
        try:
            # Try SDK first
            import pyrplidarsdk  # noqa: F401
            self._using_sdk = True
            return _SDKBackend(self.port, self.baud, self.ip, self.udp_port)
        except Exception as sdk_err:
            # Fall back to rplidar
            self._using_sdk = False
            if self.ip:
                # UDP requires SDK; explain and fail early.
                raise RuntimeError(
                    "pyrplidarsdk not available (Python 3.8). UDP mode requires SDK; "
                    "remove 'ip' or upgrade to JP6/Python 3.10+."
                ) from sdk_err
            return _RplidarBackend(self.port, self.baud, ip=None, udp_port=self.udp_port)

    def start(self):
        """Connect, check health, start scanning, spawn run loop."""
        backend = self._make_backend()
        transport = f"udp://{self.ip}:{self.udp_port}" if self._using_sdk and self.ip else f"{self.port}@{self.baud}"
        self.logger.info("RPLIDAR S2 connecting via %s (hz=%s)...", transport, self.hz)

        if not backend.connect():
            raise RuntimeError(f"RPLIDAR S2 connect failed ({transport})")

        self._driver = backend

        # Optional info/health logs
        try:
            info = self._driver.get_device_info()
            self.logger.info("RPLIDAR S2 info=%s", info)
        except Exception as e:
            self.logger.debug("get_device_info failed: %s", e)

        try:
            health = self._driver.get_health()
            self.logger.info("RPLIDAR S2 health=%s", health)
        except Exception as e:
            self.logger.debug("get_health failed: %s", e)

        if not self._driver.start_scan():
            raise RuntimeError("RPLIDAR S2 start_scan failed")

        # Warm up the data pipeline
        time.sleep(self._startup_delay)

        # Start base class and our run loop thread
        super().start()
        if not getattr(self, "_thread", None):
            self._thread = threading.Thread(
                target=self.run,
                name=f"RPLidarS2Adapter-{self.sensor_id}",
                daemon=True,
            )
            self._thread.start()

    def run(self):
        """Read scans and publish; auto-reconnect on failures."""
        self.logger.info("RPLIDAR S2 run() loop started.")
        while not self._stop.is_set():
            try:
                scan = self._driver.get_scan_data()  # (angles, ranges, qualities) or None
                now = time.time()
                if scan:
                    angles, ranges, qualities = scan
                    if len(angles) >= self._min_points:
                        if self._min_pub_period == 0.0 or (now - self._last_pub_ts) >= self._min_pub_period:
                            self.publish({
                                "sensor_id": self.sensor_id,
                                "angles": angles,
                                "ranges": ranges,
                                "qualities": qualities,
                                "timestamp": now,
                            })
                            self._last_pub_ts = now
                else:
                    if (now - self._last_empty_log) >= self._empty_log_interval and self.publish_empty_scans:
                        self.logger.debug("RPLIDAR S2: no scan data yet...")
                        self._last_empty_log = now

                # Reset backoff on successful iteration and throttle polling
                self._backoff_sec = 1.0
                time.sleep(self._poll_interval)

            except Exception as e:
                self.logger.warning("RPLIDAR S2 read error: %s; reconnect in %.1fs", e, self._backoff_sec)
                self._safe_reconnect()
                time.sleep(self._backoff_sec)
                self._backoff_sec = min(self._backoff_sec * 2, 5.0)

        self.logger.info("RPLIDAR S2 run() loop exiting.")

    def _safe_reconnect(self):
        """Stop and reconnect driver with exponential backoff."""
        if not self._driver:
            self._driver = self._make_backend()

        # stop any ongoing session
        for op in ("stop_scan", "disconnect"):
            try:
                getattr(self._driver, op)()
            except Exception:
                pass

        try:
            # rebuild backend (re-detect SDK vs rplidar)
            self._driver = self._make_backend()
            if self._driver.connect():
                if not self._driver.start_scan():
                    raise RuntimeError("start_scan failed after reconnect")
            else:
                raise RuntimeError("connect failed during reconnect")
        except Exception as e:
            self.logger.error("RPLIDAR S2 reconnect failed: %s", e)

    def stop(self):
        """Gracefully stop scanning and disconnect."""
        try:
            if self._driver:
                try:
                    self._driver.stop_scan()
                except Exception:
                    pass
                try:
                    self._driver.disconnect()
                except Exception:
                    pass
        finally:
            super().stop()


"""
RPLidar S2 Adapter (pyrplidarsdk)

Reads real scan data from a Slamtec RPLIDAR S2 using the Python wrapper
around the Slamtec SDK (pyrplidarsdk). Compatible with configs that pass `hz`:
`hz` is used to rate-limit publishing (downsample), not to change the sensor's
scan rate or motor speed.

Publishing schema (example):
{
  "sensor_id": "rplidar_s2",
  "angles": [...],      # degrees (per wrapper)
  "ranges": [...],      # units per wrapper (typically millimeters)
  "qualities": [...],
  "timestamp": 1734567890.123
}
"""

import logging
import time
import threading
from typing import Optional

from sensorhub.core.sensor_base import AbstractSensorAdapter


class RPLidarS2Adapter(AbstractSensorAdapter):
    """
    RPLidar S2 adapter using pyrplidarsdk.

    Params:
      sensor_id: Unique id for this sensor instance.
      kind:      Semantic kind (e.g., 'lidar2d').
      port:      Serial device path (Linux: /dev/ttyUSB0).
      baud:      Serial baudrate (S2 requires 1_000_000).
      ip:        Optional IP if using Slamtec network accessory (UDP).
      udp_port:  UDP port for network accessory (default: 8089).
      hz:        Optional publish rate limit (Hz). If set, downsample publishes to ~hz.
      publish_empty_scans: If True, emit empty scans when no data.
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

        # Ensure we have a logger (in case AbstractSensorAdapter doesn't set one)
        self.logger = getattr(
            self,
            "logger",
            logging.getLogger(f"sensorhub.adapters.rplidar_s2.{self.__class__.__name__}.{sensor_id}")
        )
        # Ensure we have a stop event (fallback if base class doesn't set it yet)
        self._stop = getattr(self, "_stop", threading.Event())

        self.port = port
        self.baud = baud
        self.ip = ip
        self.udp_port = udp_port
        self.hz = hz
        self.publish_empty_scans = publish_empty_scans

        # Tuning knobs
        self._startup_delay = 0.5       # seconds to let motor/sampling spin up
        self._poll_interval = 0.02      # seconds between polls (50 Hz)
        self._min_points = 500          # require at least N points before publish
        self._empty_log_interval = 2.0  # log "no data" at most once every 2s
        self._last_empty_log = 0.0

        self._driver = None
        self._thread: Optional[threading.Thread] = None
        self._backoff_sec = 1.0
        self._last_pub_ts: float = 0.0
        self._min_pub_period: float = (1.0 / hz) if (hz and hz > 0) else 0.0

    def start(self):
        """Connect, check health, start scanning, spawn run loop."""
        try:
            import pyrplidarsdk  # lazy import so tests without hardware don't fail
        except ImportError as e:
            raise RuntimeError(
                "pyrplidarsdk not installed. Run `pip install pyrplidarsdk`"
            ) from e

        # Choose transport
        if self.ip:
            self._driver = pyrplidarsdk.RplidarDriver(ip_address=self.ip, udp_port=self.udp_port)
            transport = f"udp://{self.ip}:{self.udp_port}"
        else:
            self._driver = pyrplidarsdk.RplidarDriver(port=self.port, baudrate=self.baud)
            transport = f"{self.port}@{self.baud}"

        self.logger.info("RPLIDAR S2 connecting via %s (hz=%s)...", transport, self.hz)
        if not self._driver.connect():
            raise RuntimeError(f"RPLIDAR S2 connect failed ({transport})")

        # Optional info/health logs (defensive: don't crash if methods differ)
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

        # Warm up the rotor/data pipeline
        time.sleep(self._startup_delay)

        # If base class spawning isn't guaranteed, start our own thread.
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
                    # Require a minimum number of points to avoid "partial" scans
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
                    # else: too few points yet; let it accumulate
                else:
                    # Only log "no data" occasionally to keep logs clean
                    if (now - self._last_empty_log) >= self._empty_log_interval:
                        self.logger.debug("RPLIDAR S2: no scan data yet...")
                        self._last_empty_log = now

                # Reset backoff on successful iteration and throttle polling
                self._backoff_sec = 1.0
                time.sleep(self._poll_interval)

            except Exception as e:
                self.logger.warning(
                    "RPLIDAR S2 read error: %s; reconnect in %.1fs",
                    e, self._backoff_sec
                )
                self._safe_reconnect()
                time.sleep(self._backoff_sec)
                self._backoff_sec = min(self._backoff_sec * 2, 5.0)

        self.logger.info("RPLIDAR S2 run() loop exiting.")

    def _safe_reconnect(self):
        """Stop and reconnect driver with exponential backoff."""
        for op in ("stop_scan", "disconnect"):
            try:
                getattr(self._driver, op)()
            except Exception:
                pass

        try:
            import pyrplidarsdk
            if self.ip:
                self._driver = pyrplidarsdk.RplidarDriver(ip_address=self.ip, udp_port=self.udp_port)
            else:
                self._driver = pyrplidarsdk.RplidarDriver(port=self.port, baudrate=self.baud)

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
            super().stop

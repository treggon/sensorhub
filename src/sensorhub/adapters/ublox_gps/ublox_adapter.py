
import time
import serial
import threading
import logging
from typing import Optional
from sensorhub.core.sensor_base import AbstractSensorAdapter


def _nmea_checksum_ok(line: str) -> Optional[bool]:
    if "*" not in line:
        return None
    try:
        data, hexsum = line.rsplit("*", 1)
    except ValueError:
        return False
    calc = 0
    if data.startswith("$"):
        data = data[1:]
    for ch in data:
        calc ^= ord(ch)
    try:
        expected = int(hexsum[:2], 16)
    except ValueError:
        return False
    return calc == expected


class UbloxGPSAdapter(AbstractSensorAdapter):
    """Simple u-blox adapter reading NMEA from /dev/ttyACM* or /dev/ttyUSB*.
    Emits raw NMEA lines with a timestamp and a basic checksum flag when present.
    """

    def __init__(
        self,
        sensor_id: str,
        port: str = "/dev/ttyACM0",
        baudrate: int = 9600,
        kind: str = "gps",
        hz: Optional[float] = None,     # <-- accept 'hz' from config
        **kwargs,                       # <-- swallow any future keys safely
    ) -> None:
        super().__init__(sensor_id, kind)

        self.logger = getattr(
            self,
            "logger",
            logging.getLogger(f"sensorhub.adapters.ublox.{self.__class__.__name__}.{sensor_id}"),
        )
        self._stop = getattr(self, "_stop", threading.Event())

        self.port = port
        self.baudrate = baudrate

        # Optional publish rate limit derived from hz
        self._min_pub_period = (1.0 / hz) if (hz and hz > 0) else 0.0
        self._last_pub_ts = 0.0

        self._err_backoff = 0.5  # seconds

    def run(self) -> None:
        ser = None
        try:
            ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1.0,
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False,
            )
            self.logger.info("GPS opened %s@%d", self.port, self.baudrate)
        except Exception as e:
            self.logger.error("GPS open failed on %s@%d: %s", self.port, self.baudrate, e)
            while not self._stop.is_set():
                time.sleep(self._err_backoff)
                try:
                    ser = serial.Serial(self.port, self.baudrate, timeout=1.0, write_timeout=1.0)
                    self.logger.info("GPS reopened %s@%d", self.port, self.baudrate)
                    break
                except Exception:
                    pass
            if ser is None:
                return

        try:
            while not self._stop.is_set():
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode(errors="replace").strip()
                    if not line:
                        continue

                    now = time.time()
                    if self._min_pub_period and (now - self._last_pub_ts) < self._min_pub_period:
                        # rate-limit publishes when hz is set
                        continue

                    csum = _nmea_checksum_ok(line)
                    payload = {"nmea": line, "ts": now}
                    if csum is not None:
                        payload["checksum_ok"] = bool(csum)

                    self.publish(payload)
                    self._last_pub_ts = now

                except Exception as read_err:
                    self.logger.warning(
                        "GPS read error on %s: %s; backing off %.1fs",
                        self.port, read_err, self._err_backoff
                    )
                    time.sleep(self._err_backoff)
                    try:
                        ser.close()
                    except Exception:
                        pass
                    try:
                        ser = serial.Serial(self.port, self.baudrate, timeout=1.0, write_timeout=1.0)
                        self.logger.info("GPS reopened %s@%d after error", self.port, self.baudrate)
                    except Exception as reopen_err:
                        self.logger.error("GPS reopen failed: %s", reopen_err)
                        time.sleep(self._err_backoff)

        finally:
            try:
                if ser is not None:
                    ser.close()
            except Exception:
                pass
            self.logger.info("GPS closed %s", self.port)

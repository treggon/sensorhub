
# src/sensorhub/adapters/ublox_gps/ublox_adapter.py
"""
u-blox GPS Adapter (serial NMEA/UBX)
- Robust NMEA parsing of RMC (active fix) and GGA (fix quality > 0) using comma indices.
- Converts ddmm.mmmm to decimal degrees.
- Sets `ready=True` after port open and again on first valid fix.
- Caches last fix in `_latest_frame` for /sensors/{id}/latest_raw.
- Optionally parses UBX NAV-PVT (pyubx2), but NMEA remains primary.
- Enriched health(): sats, hdop, fix quality/type, speed, track, last sentence/time.

Updated:
- __init__ now accepts 'transform: Optional[dict]' and '**kwargs', applies transform
  via adapter.set_transform() when present.
"""
import os
import time
import glob
import math
import logging
import threading
from typing import Optional, Dict, Any

import serial  # pyserial
from sensorhub.core.sensor_base import AbstractSensorAdapter

# Optional import of Transform helper (works even if not yet added to your repo)
try:
    from sensorhub.core.transform import Transform
except Exception:
    Transform = None  # type: ignore

try:
    from pyubx2 import UBXMessage, UBXReader  # optional UBX parsing
except Exception:
    UBXMessage = None  # type: ignore
    UBXReader = None   # type: ignore


# ------------------------------ helpers ------------------------------
def _find_serial_port(requested: Optional[str]) -> Optional[str]:
    if requested and os.path.exists(requested):
        return requested
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    if by_id:
        return by_id[0]
    tty_acm = sorted(glob.glob("/dev/ttyACM*"))
    if tty_acm:
        return tty_acm[0]
    tty_usb = sorted(glob.glob("/dev/ttyUSB*"))
    if tty_usb:
        return tty_usb[0]
    return requested


def _ddmm_to_deg(ddmm: Optional[str], hemi: Optional[str]) -> Optional[float]:
    """Convert NMEA ddmm.mmmm to decimal degrees."""
    if not ddmm or not hemi:
        return None
    try:
        val = float(ddmm)
        deg = math.floor(val / 100.0)
        minutes = val - (deg * 100.0)
        out = deg + minutes / 60.0
        hemi = hemi.upper()
        if hemi in ("S", "W"):
            out = -out
        return out
    except Exception:
        return None


def _to_float(v) -> Optional[float]:
    try:
        return float(v)
    except Exception:
        return None


def _knots_to_mps(knots: Optional[float]) -> Optional[float]:
    return knots * 0.514444 if knots is not None else None


# ------------------------------ adapter ------------------------------
class UbloxGPSAdapter(AbstractSensorAdapter):
    """
    Serial GPS adapter for u-blox receivers (NMEA + optional UBX).
    Produces compact fix frames for /sensors/{id}/latest_raw and enriched health().
    """

    def __init__(
        self,
        sensor_id: str,
        kind: str = "gps",
        port: Optional[str] = "/dev/ttyACM0",
        baudrate: int = 9600,
        timeout: float = 1.0,
        parse_ubx: bool = False,
        hz: Optional[float] = None,
        min_publish_interval: float = 0.25,
        log_invalid_every: float = 5.0,
        transform: Optional[Dict[str, Any]] = None,
        **kwargs,
    ) -> None:
        super().__init__(sensor_id, kind)
        self.logger = logging.getLogger(
            f"sensorhub.adapters.ublox.{self.__class__.__name__}.{sensor_id}"
        )
        self._stop = threading.Event()

        # Serial config
        self._requested_port = port
        self._port = port
        self._baud = int(baudrate)
        self._timeout = float(timeout)
        self._ser: Optional[serial.Serial] = None

        # UBX parsing (optional)
        self._parse_ubx = bool(parse_ubx) and UBXReader is not None and UBXMessage is not None
        self._ubx_reader = None

        # Publish / rate limit
        self._hz = hz
        self._min_publish_interval = float(min_publish_interval)
        self._last_pub_ts: float = 0.0

        # Latest frame cache
        self._latest_frame: Optional[Dict[str, Any]] = None

        # Logging throttles
        self._last_invalid_log_ts = 0.0
        self._log_invalid_every = float(log_invalid_every)

        # Health state
        self._sat_count: Optional[int] = None
        self._hdop: Optional[float] = None
        self._fix_quality: Optional[int] = None  # 0=invalid, 1=GPS, 2=DGPS, etc.
        self._fix_type_str: Optional[str] = None  # human-friendly
        self._speed_mps: Optional[float] = None
        self._track_deg: Optional[float] = None
        self._last_sentence: Optional[str] = None  # 'RMC' or 'GGA' or 'UBX-NAV-PVT'
        self._last_sentence_ts: Optional[float] = None

        # Absorb any extra params without crashing (future-proof)
        for k, v in kwargs.items():
            setattr(self, k, v)

        # Apply transform if provided
        if transform:
            try:
                if Transform is not None:
                    self.set_transform(Transform.from_dict(transform))
                elif hasattr(self, "set_transform"):
                    self.set_transform(transform)  # type: ignore
                else:
                    setattr(self, "_transform", transform)
            except Exception:
                pass

    # ------------------------------ public ------------------------------
    def get_latest_frame(self) -> Optional[Dict[str, Any]]:
        return self._latest_frame

    # ------------------------------ lifecycle ------------------------------
    def start(self) -> None:
        self._port = _find_serial_port(self._requested_port)
        if not self._port or not os.path.exists(self._port):
            raise RuntimeError(f"GPS port not found: {self._requested_port or '<auto>'}")
        try:
            self._ser = serial.Serial(self._port, baudrate=self._baud, timeout=self._timeout)
            self.logger.info("GPS opened %s@%d", self._port, self._baud)
        except Exception as e:
            self.last_error = f"{e.__class__.__name__}: {e}"
            raise

        # ✅ Mark ready immediately after successful open so manager registers us
        self.ready = True

        if self._parse_ubx and self._ser and UBXReader:
            try:
                self._ubx_reader = UBXReader(self._ser, validate=True, msgmode=UBXReader.MSGMODE_JSON)  # type: ignore
                self.logger.info("UBXReader enabled.")
            except Exception as e:
                self.logger.warning("UBXReader init failed; continuing with NMEA only: %s", e)
                self._ubx_reader = None

        super().start()

    def stop(self) -> None:
        try:
            if self._ser:
                try:
                    self._ser.flush()
                except Exception:
                    pass
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None
        finally:
            super().stop()

    # ------------------------------ main loop ------------------------------
    def run(self) -> None:
        self.logger.info("GPS run() loop started.")
        try:
            while not self._stop.is_set():
                if not self._ser:
                    time.sleep(0.1)
                    continue

                line = None
                try:
                    line = self._ser.readline()
                    if not line:
                        time.sleep(0.01)
                        continue
                except Exception as e:
                    self.logger.warning("GPS read error: %s", e)
                    time.sleep(0.25)
                    continue

                s = None
                try:
                    s = line.decode(errors="ignore").strip()
                except Exception:
                    s = str(line)

                # UBX parse (optional)
                if self._parse_ubx and self._ubx_reader:
                    try:
                        raw, parsed = self._ubx_reader.read()  # type: ignore
                        if parsed:
                            frame = self._frame_from_ubx(parsed)
                            if frame:
                                if not self.ready:
                                    self.ready = True
                                self._publish_if_allowed(frame)
                                continue
                    except Exception:
                        pass  # fall through to NMEA

                # NMEA parse (robust, direct by indices)
                if not s or s[0] != "$":
                    continue

                frame = self._frame_from_nmea_line(s)
                if frame:
                    # ✅ Mark ready on first valid fix
                    if not self.ready:
                        self.ready = True
                    self._publish_if_allowed(frame)
                else:
                    now = time.time()
                    if (now - self._last_invalid_log_ts) >= self._log_invalid_every:
                        self.logger.debug("Invalid/unsupported NMEA: %s", s[:80])
                        self._last_invalid_log_ts = now

        except Exception as e:
            self.logger.error("GPS run-loop error: %s", e)
            self.last_error = f"{e.__class__.__name__}: {e}"
        finally:
            self.logger.info("GPS run() loop exiting.")

    # ------------------------------ NMEA parsing ------------------------------
    def _frame_from_nmea_line(self, s: str) -> Optional[Dict[str, Any]]:
        """
        Parse $GPRMC and $GPGGA by comma positions (works across library/talker variants).
        Also updates health state (sats, hdop, fix quality, speed, track).
        """
        # Strip checksum if present
        try:
            core = s[1:]  # drop leading '$'
            star = core.find('*')
            if star != -1:
                core = core[:star]
        except Exception:
            core = s[1:] if s.startswith("$") else s

        parts = core.split(',')
        if not parts or len(parts[0]) < 5:
            return None

        talker_msg = parts[0]  # e.g., 'GPRMC', 'GPGGA', 'GNRMC', etc.
        msg = talker_msg[-3:].upper()  # 'RMC' or 'GGA'
        ts = time.time()

        # --- RMC: [0]=G?RMC,[1]=time,[2]=status,[3]=lat,[4]=NS,[5]=lon,[6]=EW,[7]=spd_knots,[8]=track_deg,...
        if msg == "RMC" and len(parts) >= 9:
            status = (parts[2] or "").upper()
            if status != "A":  # 'A' = active, 'V' = void
                return None

            lat = _ddmm_to_deg(parts[3], parts[4])
            lon = _ddmm_to_deg(parts[5], parts[6])
            spd_knots = _to_float(parts[7]) if parts[7] else None
            cog_deg = _to_float(parts[8]) if parts[8] else None
            if lat is None or lon is None:
                return None

            # Update health state
            self._speed_mps = _knots_to_mps(spd_knots)
            self._track_deg = cog_deg
            self._last_sentence = "RMC"
            self._last_sentence_ts = ts

            frame = {
                "sensor_id": self.sensor_id,
                "timestamp": ts,
                "type": "RMC",
                "lat": lat,
                "lon": lon,
                "speed_mps": self._speed_mps,
                "track_deg": self._track_deg,
            }
            self._latest_frame = frame
            return frame

        # --- GGA: [0]=G?GGA,[1]=time,[2]=lat,[3]=NS,[4]=lon,[5]=EW,[6]=fix_quality,[7]=numSV,[8]=hdop,[9]=alt_m,...
        if msg == "GGA" and len(parts) >= 10:
            # FIXED INDICES: use 6,7,8 for quality/sats/hdop
            fix_quality = _to_float(parts[6]) if parts[6] else None
            if not fix_quality or fix_quality <= 0:
                return None

            lat = _ddmm_to_deg(parts[2], parts[3])
            lon = _ddmm_to_deg(parts[4], parts[5])
            sat_count = int(parts[7]) if parts[7].isdigit() else None
            hdop = _to_float(parts[8]) if parts[8] else None
            alt_m = _to_float(parts[9]) if parts[9] else None
            if lat is None or lon is None:
                return None

            # Update health state
            self._fix_quality = int(fix_quality) if fix_quality is not None else None
            self._fix_type_str = {
                0: "invalid",
                1: "gps",
                2: "dgps",
                3: "pps",
                4: "rtk-fixed",
                5: "rtk-float",
                6: "estimated",
                7: "manual",
                8: "simulation",
            }.get(self._fix_quality, None)
            self._sat_count = sat_count
            self._hdop = hdop
            self._last_sentence = "GGA"
            self._last_sentence_ts = ts

            frame = {
                "sensor_id": self.sensor_id,
                "timestamp": ts,
                "type": "GGA",
                "lat": lat,
                "lon": lon,
                "alt_m": alt_m,
                "sats": self._sat_count,
                "hdop": self._hdop,
                "fix_quality": self._fix_quality,
            }
            self._latest_frame = frame
            return frame

        return None

    # ------------------------------ UBX parsing (optional) ------------------------------
    def _frame_from_ubx(self, parsed) -> Optional[Dict[str, Any]]:
        """
        Minimal NAV-PVT support (pyubx2). Also updates health state (sat count, speed/track).
        """
        try:
            if isinstance(parsed, dict):
                msgid = parsed.get("identity") or parsed.get("msgID")
                ts = time.time()
                if msgid and "NAV-PVT" in str(msgid):
                    lat = _to_float(parsed.get("lat"))
                    lon = _to_float(parsed.get("lon"))
                    hMSL = _to_float(parsed.get("hMSL"))
                    gSpeed = _to_float(parsed.get("gSpeed"))
                    headMot = _to_float(parsed.get("headMot"))
                    numSV = _to_float(parsed.get("numSV"))

                    if lat is not None:
                        lat /= 1e7
                    if lon is not None:
                        lon /= 1e7
                    alt = (hMSL / 1000.0) if hMSL is not None else None
                    speed_mps = (gSpeed / 1000.0) if gSpeed is not None else None
                    track_deg = (headMot / 1e5) if headMot is not None else None
                    if lat is None or lon is None:
                        return None

                    # Update health state
                    self._sat_count = int(numSV) if numSV is not None else self._sat_count
                    self._speed_mps = speed_mps
                    self._track_deg = track_deg
                    self._last_sentence = "UBX-NAV-PVT"
                    self._last_sentence_ts = ts

                    frame = {
                        "sensor_id": self.sensor_id,
                        "timestamp": ts,
                        "type": "UBX-NAV-PVT",
                        "lat": lat,
                        "lon": lon,
                        "alt_m": alt,
                        "speed_mps": self._speed_mps,
                        "track_deg": self._track_deg,
                        "sats": self._sat_count,
                    }
                    self._latest_frame = frame
                    return frame
        except Exception:
            return None
        return None

    # ------------------------------ publish gating ------------------------------
    def _publish_if_allowed(self, frame: Dict[str, Any]) -> None:
        now = time.time()
        if (now - self._last_pub_ts) < self._min_publish_interval:
            self._latest_frame = frame
            return
        # Base class publish() also flips ready=True on first publish
        self.publish(frame)
        self._latest_frame = frame
        self._last_pub_ts = now

    # ------------------------------ enriched health ------------------------------
    def health(self) -> dict:
        """
        Override: include satellites, hdop, fix quality/type, speed, track, last sentence info.
        """
        base = super().health()  # running, ready, last_error, last_sample_ts, ring_len, started_at, status
        base.update(
            {
                "sat_count": self._sat_count,
                "hdop": self._hdop,
                "fix_quality": self._fix_quality,
                "fix_type": self._fix_type_str,
                "speed_mps": self._speed_mps,
                "track_deg": self._track_deg,
                "last_sentence": self._last_sentence,
                "last_sentence_ts": self._last_sentence_ts,
            }
        )
        return base

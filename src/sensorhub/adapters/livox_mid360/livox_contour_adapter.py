
# src/sensorhub/adapters/livox_mid360/livox_contour_adapter.py
"""
Livox -> IMU-corrected -> 2.5D heightmap + static contour planes
(complete, patched version)

Capabilities
-----------
- Accumulation modes:
  * 'max' (legacy)   : per-cell top surface (highest Z).
  * 'min' (ground)   : per-cell lowest Z (ground-biased).
  * 'band' (ground+) : accept points only within a Z slice [center-half, center+half],
                       and accumulate 'min' within that slice (great for ground).

- Z-band filtering during accumulation:
  * z_band_center_m, z_band_half_m : define the accepted Z slice in robot frame.
    (In 'band' mode, half must be > 0. In 'min' mode, half>0 filters inputs before taking min.)

- Runtime config endpoints:
  * /accum/config : set accumulation mode and Z band.
  * /levels       : get/set discrete contour levels (e.g., 0.05 m spacing).
  * /transform    : get/set roll/pitch/yaw + tx/ty/tz/scale. Transform changes clear the map.
  * /decay        : manual decay of hits; /decay/config for periodic/alpha settings.
  * /clear        : clear heightmap + hits.

- Image exporters:
  * /heightmap.png
      scale_mode = 'levels' (default), 'auto', 'equalize', 'fixed'
      valid-mask scaling (ignore NaN for contrast)
      optional hillshade overlay
      windowed export via x_min/x_max/y_min/y_max (meters)
      default window is centered ±default_window_radius_m (default: 8 m)

  * /contours.png
      draws quantized bands + thicker isolines (configurable line width/color/alpha)
      windowed export

- Binary + OBJ exporters:
  * /heightmap.raw           : float32 (nx*ny) row-major
  * /contour_planes.obj      : fast, one quad per level with per-level material
  * /contours.obj            : OBJ line elements per isoline

- Metadata:
  * /meta                     : dims, bounds, levels, valid_ratio, accum config, default window, transform

Defaults
--------
- Levels: every 0.2 m from -0.5 .. +2.0 m
- Heightmap PNG: 'levels' scale_mode, centered ±8 m window
- Contours PNG: white lines, width=3 px

Health model (adapter-level)
----------------------------
- ok      : fresh map updates in < ok_stale_sec
- warning : running but no recent map updates (< err_stale_sec) OR upstream warning
- error   : adapter not running OR upstream error/unavailable OR map stale ≥ err_stale_sec
"""

import math
import time
import threading
import logging
import json
from typing import Dict, Tuple, Optional, List

import numpy as np
from fastapi import APIRouter, HTTPException, Response

from sensorhub.core.sensor_base import AbstractSensorAdapter
from sensorhub.core.sensor_manager import manager

try:
    from PIL import Image, ImageDraw, ImageOps
    PIL_OK = True
except Exception:
    PIL_OK = False

router = APIRouter(prefix="/livox_contour", tags=["livox_contour"])


class LivoxContourAdapter(AbstractSensorAdapter):
    def __init__(
        self,
        sensor_id: str,
        kind: str = "contours",
        source_id: str = "livox",
        cell_size_m: float = 0.05,
        grid_xy_m: float = 20.0,
        z_min_m: float = -2.0,
        z_max_m: float = 8.0,
        publish_hz: Optional[float] = 2.0,
        decay_enable: bool = True,
        decay_alpha: float = 0.98,
        decay_period_s: float = 5.0,
        levels_m: Optional[List[float]] = None,
        default_window_radius_m: float = 8.0,
        # Accumulation behavior
        accum_mode: str = "max",        # 'max'|'min'|'band'
        z_band_center_m: float = 0.0,   # used in 'band' mode; if half>0 also filters 'min'
        z_band_half_m: float = 0.0,     # 0 disables band filter unless mode=='band'
        # --- NEW health knobs (same semantics as Livox adapter) ---
        ok_stale_sec: float = 1.0,
        warn_stale_sec: float = 3.0,
        err_stale_sec: float = 10.0,
        **kwargs,
    ):
        super().__init__(sensor_id, kind)
        self.log = logging.getLogger(f"sensorhub.adapters.livox_contour.{sensor_id}")
        self._stop = threading.Event()

        # Source adapter id (Livox)
        self.source_id = source_id

        # Grid geometry (robot frame)
        self.cell = float(cell_size_m)
        self.xy = float(grid_xy_m)
        self.xmin, self.xmax = -self.xy, +self.xy
        self.ymin, self.ymax = -self.xy, +self.xy
        self.nx = int(math.ceil((self.xmax - self.xmin) / self.cell))
        self.ny = int(math.ceil((self.ymax - self.ymin) / self.cell))

        # Z bounds (clamp)
        self.zmin = float(z_min_m)
        self.zmax = float(z_max_m)

        # Data arrays
        self.H = np.full((self.nx, self.ny), np.nan, dtype=np.float32)   # height (NaN=unknown)
        self.hits = np.zeros((self.nx, self.ny), dtype=np.uint16)        # # of accepted hits per cell

        # Transform (robot frame)
        self.transform: Dict[str, float] = {
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
            "tx": 0.0, "ty": 0.0, "tz": 0.0, "scale": 1.0
        }

        # Optional IMU
        self._imu: Optional[Dict[str, float]] = None
        self.use_gyro_rotation = False
        self.imu_eps = 0.01
        self.last_imu_ts: Optional[float] = None

        # Publish cadence
        self.publish_period = (1.0 / publish_hz) if (publish_hz and publish_hz > 0.0) else 0.5
        self._last_pub = time.time()

        # Decay
        self.decay_enable = bool(decay_enable)
        self.decay_alpha = float(decay_alpha)
        self.decay_period_s = float(decay_period_s)
        self._last_decay = time.time()

        # Contour levels
        if levels_m is None:
            self.levels = [round(v, 3) for v in np.arange(-0.5, 2.05, 0.2).tolist()]
        else:
            self.levels = [float(v) for v in levels_m]

        # Default image window radius (meters)
        self.default_window_radius_m = float(default_window_radius_m)

        # Accumulation config
        self.accum_mode = str(accum_mode).lower()
        self.z_band_center_m = float(z_band_center_m)
        self.z_band_half_m = float(z_band_half_m)

        # --- NEW: health tracking ---
        self.ok_stale_sec = float(ok_stale_sec)
        self.warn_stale_sec = float(warn_stale_sec)
        self.err_stale_sec = float(err_stale_sec)
        self._last_map_ts: float = 0.0          # time.time() of last successful map update
        self._last_source_ts: float = 0.0       # timestamp from upstream sample (if provided)

    # ---------- lifecycle ----------
    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        self._stop.set()
        super().stop()

    # ---------- transform setter (clear map on change) ----------
    def set_transform(self, t) -> None:
        """
        Accepts either a dict-like or an object with attributes roll_deg/pitch_deg/yaw_deg/tx/ty/tz/scale.
        Clears the map on change (existing behavior).
        """
        def _get(src, key, default):
            # Works for dicts and objects (e.g., dataclass Transform)
            if isinstance(src, dict):
                return src.get(key, default)
            return getattr(src, key, default)

        self.transform = {
            "roll_deg": float(_get(t, "roll_deg", 0.0)),
            "pitch_deg": float(_get(t, "pitch_deg", 0.0)),
            "yaw_deg": float(_get(t, "yaw_deg", 0.0)),
            "tx": float(_get(t, "tx", 0.0)),
            "ty": float(_get(t, "ty", 0.0)),
            "tz": float(_get(t, "tz", 0.0)),
            "scale": float(_get(t, "scale", 1.0)),
        }
        self.H[:] = np.nan
        self.hits[:] = 0
        self._last_map_ts = 0.0  # keep health coherent after clear
        self.log.info("Transform updated; heightmap cleared.")

    # ---------- transforms / IMU ----------
    def _apply_transform(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        t = self.transform or {}
        try:
            sx = float(t.get("scale", 1.0))
            rx = math.radians(float(t.get("roll_deg", 0.0)))
            ry = math.radians(float(t.get("pitch_deg", 0.0)))
            rz = math.radians(float(t.get("yaw_deg", 0.0)))
            tx = float(t.get("tx", 0.0)); ty = float(t.get("ty", 0.0)); tz = float(t.get("tz", 0.0))
            x, y, z = sx * x, sx * y, sx * z
            # Rz
            cz, sz = math.cos(rz), math.sin(rz)
            x, y = (cz * x - sz * y), (sz * x + cz * y)
            # Ry
            cy, sy = math.cos(ry), math.sin(ry)
            x, z = (cy * x + sy * z), (-sy * x + cy * z)
            # Rx
            cx, sx_ = math.cos(rx), math.sin(rx)
            y, z = (cx * y - sx_ * z), (sx_ * y + cx * z)
            return x + tx, y + ty, z + tz
        except Exception:
            return x, y, z

    def _imu_rotation_quat(self) -> Tuple[float, float, float, float]:
        if not (self.use_gyro_rotation and self._imu):
            return (1.0, 0.0, 0.0, 0.0)
        gx = float(self._imu.get("gx_radps", 0.0))
        gy = float(self._imu.get("gy_radps", 0.0))
        gz = float(self._imu.get("gz_radps", 0.0))
        ts = self._imu.get("ts_sec")
        if ts is None or self.last_imu_ts is None:
            self.last_imu_ts = ts
            return (1.0, 0.0, 0.0, 0.0)
        dt = max(0.0, float(ts) - float(self.last_imu_ts))
        self.last_imu_ts = ts
        omega = math.sqrt(gx * gx + gy * gy + gz * gz)
        if omega < self.imu_eps or dt <= 0.0:
            return (1.0, 0.0, 0.0, 0.0)
        hdt = 0.5 * dt
        return (1.0, hdt * gx, hdt * gy, hdt * gz)

    def _apply_quat(self, x: float, y: float, z: float, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        _, qx, qy, qz = q
        vx, vy, vz = x, y, z
        cx = qy * vz - qz * vy
        cy = qz * vx - qx * vz
        cz = qx * vy - qy * vx
        return vx + 2.0 * cx, vy + 2.0 * cy, vz + 2.0 * cz

    # ---------- accumulation (modes) ----------
    def _insert_height(self, xr: float, yr: float, zr: float, min_hits_for_update: int = 1) -> None:
        # Bounds
        if not (self.xmin <= xr <= self.xmax and self.ymin <= yr <= self.ymax and self.zmin <= zr <= self.zmax):
            return

        # Z-band filter (if configured)
        if self.accum_mode in ("band", "min") and self.z_band_half_m > 0.0:
            lo = self.z_band_center_m - self.z_band_half_m
            hi = self.z_band_center_m + self.z_band_half_m
            if zr < lo or zr > hi:
                return
        elif self.accum_mode == "band":
            # 'band' mode requires non-zero half to accept anything
            return

        ix = int((xr - self.xmin) / self.cell)
        iy = int((yr - self.ymin) / self.cell)

        v_prev = self.H[ix, iy]
        self.hits[ix, iy] = min(np.uint16(65535), np.uint16(self.hits[ix, iy] + 1))

        mode = self.accum_mode
        if mode == "min":
            if not math.isfinite(v_prev) or (zr < float(v_prev)):
                self.H[ix, iy] = float(zr)
        elif mode == "band":
            if not math.isfinite(v_prev) or (zr < float(v_prev)):
                self.H[ix, iy] = float(zr)
        else:  # "max" (legacy top surface)
            if not math.isfinite(v_prev) or (zr > float(v_prev)):
                self.H[ix, iy] = float(zr)

    def decay(self, alpha: Optional[float] = None, min_hits_keep: int = 1) -> Dict[str, int]:
        a = float(self.decay_alpha if alpha is None else alpha)
        a = max(0.0, min(1.0, a))
        h = self.hits.astype(np.float32) * a
        self.hits[:] = h.astype(np.uint16)
        mask_clear = self.hits < int(min_hits_keep)
        cleared = int(np.count_nonzero(mask_clear))
        if cleared > 0:
            self.H[mask_clear] = np.nan
        kept = int(self.nx * self.ny - cleared)
        return {"kept": kept, "cleared": cleared}

    def run(self) -> None:
        self.log.info("LivoxContourAdapter run() loop.")
        try:
            while not self._stop.is_set():
                sample = manager.latest(self.source_id)
                if sample and isinstance(sample.data, dict):
                    # Track upstream sensor timestamp if provided
                    src_ts = sample.data.get("timestamp")
                    if isinstance(src_ts, (int, float)):
                        self._last_source_ts = float(src_ts)

                    pts = sample.data.get("points") or []
                    imu = sample.data.get("imu")
                    if imu:
                        self._imu = imu
                    q = (1.0, 0.0, 0.0, 0.0) if not self.use_gyro_rotation else self._imu_rotation_quat()

                    any_update = False
                    for p in pts:
                        if len(p) < 2:
                            continue
                        if len(p) == 2:
                            x, y = float(p[0]), float(p[1]); z = 0.0
                        elif len(p) == 3:
                            x, y, z = float(p[0]), float(p[1]), float(p[2])
                        else:
                            x, y, z = float(p[0]), float(p[1]), float(p[2])
                        x, y, z = self._apply_quat(x, y, z, q)
                        xr, yr, zr = self._apply_transform(x, y, z)
                        self._insert_height(xr, yr, zr)
                        any_update = True

                    if any_update:
                        self._last_map_ts = time.time()

                now = time.time()
                if (now - self._last_pub) >= self.publish_period:
                    self.publish({
                        "sensor_id": self.sensor_id,
                        "status": "running",
                        "timestamp": now,
                        "dims": (self.nx, self.ny),
                        "cell_size_m": self.cell,
                        "bounds": {
                            "xmin": self.xmin, "xmax": self.xmax,
                            "ymin": self.ymin, "ymax": self.ymax,
                            "zmin": self.zmin, "zmax": self.zmax
                        },
                        "levels": self.levels,
                        "valid_ratio": self._valid_ratio(),
                        "accum": {
                            "mode": self.accum_mode,
                            "z_band_center_m": self.z_band_center_m,
                            "z_band_half_m": self.z_band_half_m
                        },
                        "default_window_radius_m": self.default_window_radius_m
                    })
                    self._last_pub = now

                try:
                    if self.decay_enable and (time.time() - self._last_decay) >= float(self.decay_period_s):
                        _ = self.decay(self.decay_alpha)
                        self._last_decay = time.time()
                except Exception:
                    pass

                time.sleep(0.01)
        finally:
            self.log.info("LivoxContourAdapter exit.")

    def _valid_ratio(self) -> float:
        total = self.nx * self.ny
        valid = int(np.count_nonzero(np.isfinite(self.H)))
        return valid / max(1, total)

    # =========================
    # NEW: health & readiness
    # =========================
    def _last_update_age_sec(self) -> Optional[float]:
        """
        Age (seconds) since the heightmap was last updated from upstream points.
        None => no updates have ever occurred.
        """
        if self._last_map_ts <= 0.0:
            return None
        return max(0.0, time.time() - self._last_map_ts)

    def is_ready(self) -> bool:
        """
        Ready when we have a recent update (you can also require some valid coverage if desired).
        """
        age = self._last_update_age_sec()
        return (age is not None) and (age < self.warn_stale_sec)

    def health(self) -> dict:
        """
        Report adapter health with ok/warning/error classification.
        Takes upstream (source_id) status into account but does not hard-fail
        unless upstream is explicitly in 'error' OR local map is long-stale.
        """
        now = time.time()
        running = self.is_running()
        age = self._last_update_age_sec()     # None if never updated
        vr = self._valid_ratio()

        # Inspect upstream sensor (e.g., Livox adapter)
        upstream = None
        upstream_status = None
        try:
            upstream = manager.get_status(self.source_id)
            if isinstance(upstream, dict):
                upstream_status = upstream.get("status")
        except Exception:
            upstream_status = None

        # Classify
        if not running:
            status = "error"
            reason = "adapter not running"
        else:
            if upstream_status == "error":
                # Upstream sensor in error: escalate unless we still have very fresh map (rare).
                if age is None or age >= self.ok_stale_sec:
                    status = "error"
                    reason = "upstream error"
                else:
                    status = "warning"
                    reason = "upstream error; using cached map"
            else:
                # Upstream ok/warning/unknown; base on our own staleness
                if age is None:
                    # Never updated: if upstream looks ok or warning -> yellow; else red.
                    if upstream_status in ("ok", "warning", None):
                        status = "warning"
                        reason = "no map updates yet; awaiting points"
                    else:
                        status = "error"
                        reason = f"upstream {upstream_status}"
                elif age < self.ok_stale_sec:
                    status = "ok"
                    reason = "fresh map"
                elif age < self.err_stale_sec:
                    status = "warning"
                    reason = f"map stale ({age:.1f}s)"
                else:
                    status = "error"
                    reason = f"no map updates for {age:.1f}s"

        return {
            "id": self.sensor_id,
            "kind": self.kind,
            "status": status,                     # ui: green / yellow / red
            "reason": reason,
            "running": running,
            "ready": (status == "ok"),
            "last_update_ts": self._last_map_ts if self._last_map_ts > 0 else None,
            "last_data_age_sec": None if age is None else round(age, 2),
            "valid_ratio": round(vr, 4),
            "upstream": upstream if isinstance(upstream, dict) else None,
            "timestamp": now,
        }


# ---------- REST helpers & routes ----------

def _get_contour_adapter(adapter_id: str = "livox_contour") -> LivoxContourAdapter:
    a = manager.get_adapter(adapter_id)
    if not a or not isinstance(a, LivoxContourAdapter):
        raise HTTPException(404, "contour adapter not found")
    return a


@router.get("/routes")
def list_routes(adapter_id: str = "livox_contour"):
    return {
        "adapter_id": adapter_id,
        "routes": {
            "meta": "/livox_contour/meta",
            "heightmap_raw": "/livox_contour/heightmap.raw",
            "heightmap_png": "/livox_contour/heightmap.png",
            "contours_png": "/livox_contour/contours.png",
            "contours_obj": "/livox_contour/contours.obj",
            "planes_obj": "/livox_contour/contour_planes.obj",
            "levels_get": "/livox_contour/levels",
            "levels_set": "/livox_contour/levels",
            "transform_get": "/livox_contour/transform",
            "transform_set": "/livox_contour/transform",
            "clear": "/livox_contour/clear",
            "decay_config_get": "/livox_contour/decay/config",
            "decay_config_set": "/livox_contour/decay/config",
            "decay_once": "/livox_contour/decay",
            "accum_config": "/livox_contour/accum/config",
        }
    }


@router.get("/meta")
def contour_meta(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    return {
        "dims": (a.nx, a.ny),
        "cell_size_m": a.cell,
        "bounds": {"xmin": a.xmin, "xmax": a.xmax, "ymin": a.ymin, "ymax": a.ymax, "zmin": a.zmin, "zmax": a.zmax},
        "levels": a.levels,
        "valid_ratio": a._valid_ratio(),
        "accum": {"mode": a.accum_mode, "z_band_center_m": a.z_band_center_m, "z_band_half_m": a.z_band_half_m},
        "default_window_radius_m": a.default_window_radius_m,
        "transform": a.transform
    }


# ---- Accumulation config (mode + Z-band) ----
@router.post("/accum/config")
def set_accum_config(
    adapter_id: str = "livox_contour",
    mode: Optional[str] = None,               # 'max'|'min'|'band'
    z_band_center_m: Optional[float] = None,
    z_band_half_m: Optional[float] = None
):
    a = _get_contour_adapter(adapter_id)
    if mode is not None:
        m = str(mode).lower()
        if m not in ("max", "min", "band"):
            raise HTTPException(400, "mode must be one of: max|min|band")
        a.accum_mode = m
    if z_band_center_m is not None:
        a.z_band_center_m = float(z_band_center_m)
    if z_band_half_m is not None:
        a.z_band_half_m = max(0.0, float(z_band_half_m))
    return {"ok": True, "accum": {"mode": a.accum_mode, "z_band_center_m": a.z_band_center_m, "z_band_half_m": a.z_band_half_m}}


# ---- Levels ----
@router.get("/levels")
def get_levels(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    return {"ok": True, "levels": a.levels}


@router.post("/levels")
def set_levels(adapter_id: str = "livox_contour", levels_json: Optional[str] = None):
    a = _get_contour_adapter(adapter_id)
    if levels_json:
        try:
            levels = json.loads(levels_json)
            a.levels = [float(v) for v in levels]
        except Exception as e:
            raise HTTPException(400, f"invalid levels_json: {e}")
    return {"ok": True, "levels": a.levels}


# ---- Transform ----
@router.get("/transform")
def get_transform(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    return {"ok": True, "sensor_id": adapter_id, "transform": a.transform}


@router.post("/transform")
def set_transform(
    adapter_id: str = "livox_contour",
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
    scale: float = 1.0,
):
    a = _get_contour_adapter(adapter_id)
    a.set_transform({"roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg, "tx": tx, "ty": ty, "tz": tz, "scale": scale})
    return {"ok": True, "sensor_id": adapter_id, "transform": a.transform}


# ---- Clear & Decay ----
@router.post("/clear")
def clear(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    a.H[:] = np.nan
    a.hits[:] = 0
    a._last_map_ts = 0.0
    return {"status": "ok"}


@router.get("/decay/config")
def decay_config(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    return {"enable_decay": bool(a.decay_enable), "decay_alpha": float(a.decay_alpha), "decay_period_s": float(a.decay_period_s)}


@router.post("/decay/config")
def set_decay_config(
    adapter_id: str = "livox_contour",
    enable_decay: Optional[int] = None,
    decay_alpha: Optional[float] = None,
    decay_period_s: Optional[float] = None
):
    a = _get_contour_adapter(adapter_id)
    if enable_decay is not None:
        a.decay_enable = bool(int(enable_decay))
    if decay_alpha is not None:
        a.decay_alpha = float(decay_alpha)
    if decay_period_s is not None:
        a.decay_period_s = float(decay_period_s)
    return decay_config(adapter_id)


@router.post("/decay")
def decay_once(adapter_id: str = "livox_contour", alpha: float = 0.98, min_hits_keep: int = 1):
    a = _get_contour_adapter(adapter_id)
    stats = a.decay(alpha=alpha, min_hits_keep=min_hits_keep)
    return {"status": "ok", "alpha": alpha, "min_hits_keep": min_hits_keep, "stats": stats}


# ---- Binary Export ----
@router.get("/heightmap.raw", response_class=Response)
def heightmap_raw(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    raw = a.H.astype(np.float32).tobytes(order="C")
    headers = {"X-Width": str(a.nx), "X-Height": str(a.ny), "X-Cell-M": str(a.cell)}
    return Response(content=raw, media_type="application/octet-stream", headers=headers)


# ---- Heightmap PNG ----
@router.get("/heightmap.png", response_class=Response)
def heightmap_png(
    adapter_id: str = "livox_contour",
    scale_mode: str = "levels",  # 'levels' (default) | 'auto' | 'equalize' | 'fixed'
    gain: float = 1.0,
    clip_min_m: Optional[float] = None,
    clip_max_m: Optional[float] = None,
    cmap: str = "viridis",
    draw_grid: int = 0,
    tick_m: float = 1.0,
    mark_center: int = 1,
    invert_y: int = 0,
    downscale: int = 2,
    # Window (DEFAULT: centered ±default_window_radius_m if any is None)
    x_min: Optional[float] = None,
    x_max: Optional[float] = None,
    y_min: Optional[float] = None,
    y_max: Optional[float] = None,
    # Hillshade overlay
    hillshade: int = 0,
    azimuth_deg: float = 315.0,
    elevation_deg: float = 35.0,
):
    a = _get_contour_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")

    # Resolve default window if not provided
    if x_min is None or x_max is None or y_min is None or y_max is None:
        r = float(getattr(a, "default_window_radius_m", 8.0))
        x_min, x_max = -r, +r
        y_min, y_max = -r, +r

    H = a.H.copy()
    ix0, ix1, iy0, iy1 = _window_indices(a, x_min, x_max, y_min, y_max)
    H = H[ix0 : ix1 + 1, iy0 : iy1 + 1]
    valid = np.isfinite(H)

    if not valid.any():
        H_vis = np.full(H.shape, a.zmin, dtype=np.float32)
        arr = np.zeros_like(H_vis, dtype=np.float32)
    else:
        H_vis = H.copy()
        if clip_min_m is not None or clip_max_m is not None:
            lo = a.zmin if clip_min_m is None else float(clip_min_m)
            hi = a.zmax if clip_max_m is None else float(clip_max_m)
            H_vis[valid] = np.clip(H_vis[valid], lo, hi)

        # Scaling modes
        if scale_mode == "auto":
            vals = H_vis[valid]
            p5, p95 = np.percentile(vals, 5), np.percentile(vals, 95)
            arr = (H_vis - p5) * (255.0 / max(1e-6, p95 - p5))
        elif scale_mode == "equalize":
            arr = (H_vis - a.zmin) * (255.0 / max(1e-3, (a.zmax - a.zmin))) * gain
            arr = np.clip(arr, 0.0, 255.0)
            arr_u8 = arr.astype(np.uint8)
            img_tmp = Image.fromarray(arr_u8, mode="L")
            try:
                img_tmp = ImageOps.equalize(img_tmp)
            except Exception:
                pass
            arr = np.array(img_tmp)
        elif scale_mode == "levels":
            lv = np.array(a.levels, dtype=np.float32)
            arr = np.zeros_like(H_vis, dtype=np.float32)
            if lv.size > 0:
                z = H_vis[valid][:, None]
                dif = np.abs(z - lv[None, :])
                idx = np.argmin(dif, axis=1)
                arr[valid] = (idx.astype(np.float32) / max(1, lv.size - 1)) * 255.0
        else:  # 'fixed'
            arr = (H_vis - a.zmin) * (255.0 / max(1e-3, (a.zmax - a.zmin))) * gain

        arr[~valid] = 0.0
        arr = np.clip(arr, 0.0, 255.0)

    img = Image.fromarray(arr.astype(np.uint8), mode="L")

    # Optional hillshade overlay
    if int(hillshade) == 1 and valid.any():
        hs = _hillshade(H, valid, a.cell, azimuth_deg, elevation_deg)
        img_hs = Image.fromarray(np.clip(hs, 0, 255).astype(np.uint8), mode="L")
        img = Image.blend(img.convert("RGB"), img_hs.convert("RGB"), alpha=0.3)

    # Colormap
    img = _apply_cmap(img, cmap)

    # Invert Y?
    if invert_y:
        img = ImageOps.flip(img)

    # Overlays
    draw = ImageDraw.Draw(img)
    w, h = img.size
    if draw_grid:
        spacing_px = int(round(tick_m / a.cell))
        if spacing_px >= 1:
            col = (200, 200, 200)
            for x in range(0, w, spacing_px):
                draw.line([(x, 0), (x, h - 1)], fill=col, width=1)
            for y in range(0, h, spacing_px):
                draw.line([(0, y), (w - 1, y)], fill=col, width=1)

    if mark_center:
        cx = int(round((0.0 - x_min) / a.cell))
        cy = int(round((0.0 - y_min) / a.cell))
        cx = max(0, min(w - 1, cx))
        cy = max(0, min(h - 1, cy))
        draw.line([(cx - 6, cy), (cx + 6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy - 6), (cx, cy + 6)], fill=(255, 0, 0), width=2)

    if downscale and int(downscale) > 1:
        factor = int(downscale)
        img = img.resize((max(1, w // factor), max(1, h // factor)), resample=Image.BILINEAR)

    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {
        "X-Width": str(img.size[0]),
        "X-Height": str(img.size[1]),
        "X-Cell-M": str(a.cell),
        "X-Window": json.dumps({"x": [x_min, x_max], "y": [y_min, y_max]}),
        "X-Scale-Mode": scale_mode,
    }
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)


# ---- Contours PNG (bands + isolines) ----
@router.get("/contours.png", response_class=Response)
def contours_png(
    adapter_id: str = "livox_contour",
    cmap: str = "viridis",
    line_color: str = "white",
    line_alpha: float = 0.95,
    line_width_px: int = 3,
    downscale: int = 2,
    # Window (DEFAULT: centered ±default_window_radius_m if any is None)
    x_min: Optional[float] = None,
    x_max: Optional[float] = None,
    y_min: Optional[float] = None,
    y_max: Optional[float] = None,
):
    a = _get_contour_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")

    # Resolve default window if not provided
    if x_min is None or x_max is None or y_min is None or y_max is None:
        r = float(getattr(a, "default_window_radius_m", 8.0))
        x_min, x_max = -r, +r
        y_min, y_max = -r, +r

    # Base bands image (quantized to levels for clear steps)
    base = heightmap_png(
        adapter_id=adapter_id,
        scale_mode="levels",
        cmap=cmap,
        downscale=1,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        mark_center=1,
    )
    try:
        from io import BytesIO
        img = Image.open(BytesIO(base.body))
    except Exception as e:
        raise HTTPException(500, f"failed to build base image: {e}")

    draw = ImageDraw.Draw(img)
    col = (255, 255, 255) if line_color.lower() == "white" else (255, 0, 0)

    # Windowed contours
    H = a.H.copy()
    ix0, ix1, iy0, iy1 = _window_indices(a, x_min, x_max, y_min, y_max)
    subH = H[ix0 : ix1 + 1, iy0 : iy1 + 1]
    contours = _contours_from_height(subH, a.levels, a.xmin + ix0 * a.cell, a.ymin + iy0 * a.cell, a.cell)

    for L, lines in contours.items():
        rgba = (*col, int(max(0, min(255, round(255.0 * line_alpha)))))
        for line in lines:
            pts_px = []
            for (x, y, _z) in line:
                px = int(round((x - x_min) / a.cell))
                py = int(round((y - y_min) / a.cell))
                pts_px.append((px, py))
            if len(pts_px) >= 2:
                draw.line(pts_px, fill=rgba, width=max(1, int(line_width_px)))

    if downscale and int(downscale) > 1:
        factor = int(downscale)
        img = img.resize((img.size[0] // factor, img.size[1] // factor), resample=Image.BILINEAR)

    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {"X-Width": str(img.size[0]), "X-Height": str(img.size[1]), "X-Cell-M": str(a.cell)}
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)


# ---- OBJ exports ----
@router.get("/contour_planes.obj", response_class=Response)
def contour_planes_obj(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    mtl_name = "contour_planes.mtl"
    obj_lines = []
    mtl_lines = ["# contour planes materials\n"]

    def level_color(k: int, n: int) -> Tuple[int, int, int]:
        t = k / max(1, n - 1)
        r = int(68 + 187 * t)
        g = int(1 + 255 * t)
        b = int(84 + 140 * t)
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        return r, g, b

    v_idx = 1
    obj_lines.append(f"mtllib {mtl_name}\n")
    for i, L in enumerate(a.levels):
        r, g, b = level_color(i, len(a.levels))
        mtl = f"level_{i}"
        mtl_lines.append(f"newmtl {mtl}\nKd {r/255.0:.4f} {g/255.0:.4f} {b/255.0:.4f}\n\n")
        # One quad per level spanning the full grid window
        v = [(a.xmin, a.ymin, L), (a.xmax, a.ymin, L), (a.xmax, a.ymax, L), (a.xmin, a.ymax, L)]
        for (x, y, z) in v:
            obj_lines.append(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        obj_lines.append(f"usemtl {mtl}\n")
        obj_lines.append(f"f {v_idx} {v_idx+1} {v_idx+2} {v_idx+3}\n")
        v_idx += 4

    obj = "".join(obj_lines)
    mtl = "".join(mtl_lines)
    headers = {"X-MTL-Filename": mtl_name, "X-Levels": json.dumps(a.levels)}
    return Response(content=obj.encode("ascii"), media_type="text/plain", headers=headers)


@router.get("/contours.obj", response_class=Response)
def contours_obj(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    H = a.H.copy()
    contours = _contours_from_height(H, a.levels, a.xmin, a.ymin, a.cell)
    obj_lines = []
    obj_lines.append("# contour isolines\n")
    obj_lines.append("o contours\n")
    v_idx = 1
    for i, L in enumerate(a.levels):
        obj_lines.append(f"g level_{i}\n")
        for poly in contours.get(L, []):
            idxs = []
            for (x, y, z) in poly:
                obj_lines.append(f"v {x:.5f} {y:.5f} {z:.5f}\n")
                idxs.append(v_idx)
                v_idx += 1
            if len(idxs) >= 2:
                obj_lines.append("l " + " ".join(str(k) for k in idxs) + "\n")
    obj = "".join(obj_lines)
    headers = {"X-Levels": json.dumps(a.levels)}
    return Response(content=obj.encode("ascii"), media_type="text/plain", headers=headers)


# ---------- helpers ----------

def _apply_cmap(img: "Image.Image", cmap: str) -> "Image.Image":
    """
    Simple built-in LUTs (viridis, hot). Converts grayscale to RGB.
    """
    lut = None
    if cmap.lower() == "hot":
        lut = []
        for i in range(256):
            r = min(255, int(i * 1.2))
            g = min(255, int(max(0, i - 64) * 1.2))
            b = min(255, int(max(0, i - 128) * 1.2))
            lut += [r, g, b]
    elif cmap.lower() == "viridis":
        lut = []
        for i in range(256):
            t = i / 255.0
            r = int(68 + 187 * t)
            g = int(1 + 255 * t)
            b = int(84 + 140 * t)
            r = max(0, min(255, r))
            g = max(0, min(255, g))
            b = max(0, min(255, b))
            lut += [r, g, b]
    if lut:
        img = img.convert("P")
        img.putpalette(lut)
        img = img.convert("RGB")
    else:
        img = img.convert("RGB")
    return img


def _window_indices(a: LivoxContourAdapter, x_min: float, x_max: float, y_min: float, y_max: float) -> Tuple[int, int, int, int]:
    """
    Clamp a world window to the adapter grid and return grid indices.
    """
    xmin = max(a.xmin, float(x_min))
    xmax = min(a.xmax, float(x_max))
    ymin = max(a.ymin, float(y_min))
    ymax = min(a.ymax, float(y_max))
    if xmin >= xmax:
        xmin, xmax = a.xmin, a.xmax
    if ymin >= ymax:
        ymin, ymax = a.ymin, a.ymax
    ix0 = int((xmin - a.xmin) / a.cell)
    ix1 = int((xmax - a.xmin) / a.cell)
    iy0 = int((ymin - a.ymin) / a.cell)
    iy1 = int((ymax - a.ymin) / a.cell)
    ix0 = max(0, min(a.nx - 1, ix0))
    ix1 = max(0, min(a.nx - 1, ix1))
    iy0 = max(0, min(a.ny - 1, iy0))
    iy1 = max(0, min(a.ny - 1, iy1))
    if ix1 < ix0:
        ix0, ix1 = ix1, ix0
    if iy1 < iy0:
        iy0, iy1 = iy1, iy0
    return ix0, ix1, iy0, iy1


def _contours_from_height(H: np.ndarray, levels: List[float], x0: float, y0: float, cell: float) -> Dict[float, List[List[Tuple[float, float, float]]]]:
    """
    Marching squares over a (sub)heightmap.
    Return dict: level -> list of polylines (each: list of (x,y,z=level)).
    """
    out: Dict[float, List[List[Tuple[float, float, float]]]] = {}
    nx, ny = H.shape[0], H.shape[1]
    for L in levels:
        lines: List[List[Tuple[float, float, float]]] = []
        for ix in range(nx - 1):
            for iy in range(ny - 1):
                z00 = H[ix, iy]
                z10 = H[ix + 1, iy]
                z01 = H[ix, iy + 1]
                z11 = H[ix + 1, iy + 1]
                if not (math.isfinite(z00) or math.isfinite(z10) or math.isfinite(z01) or math.isfinite(z11)):
                    continue
                # Binary mask >= L
                b0 = 1 if (math.isfinite(z00) and z00 >= L) else 0
                b1 = 1 if (math.isfinite(z10) and z10 >= L) else 0
                b2 = 1 if (math.isfinite(z11) and z11 >= L) else 0
                b3 = 1 if (math.isfinite(z01) and z01 >= L) else 0
                code = (b0 << 0) | (b1 << 1) | (b2 << 2) | (b3 << 3)
                if code == 0 or code == 0xF:
                    continue

                def lerp(a: float, b: float, va: float, vb: float, L: float) -> float:
                    if not (math.isfinite(va) and math.isfinite(vb)):
                        return 0.5 * (a + b)
                    if abs(vb - va) < 1e-9:
                        return 0.5 * (a + b)
                    t = (L - va) / (vb - va)
                    return a + t * (b - a)

                x00 = x0 + ix * cell
                x10 = x0 + (ix + 1) * cell
                y00 = y0 + iy * cell
                y01 = y0 + (iy + 1) * cell
                edges: List[Tuple[float, float]] = []
                if (b0 != b1):  # bottom
                    edges.append((lerp(x00, x10, z00, z10, L), y00))
                if (b1 != b2):  # right
                    edges.append((x10, lerp(y00, y01, z10, z11, L)))
                if (b3 != b2):  # top
                    edges.append((lerp(x00, x10, z01, z11, L), y01))
                if (b0 != b3):  # left
                    edges.append((x00, lerp(y00, y01, z00, z01, L)))
                if len(edges) >= 2:
                    lines.append([(edges[0][0], edges[0][1], L), (edges[1][0], edges[1][1], L)])
        out[L] = lines
    return out


def _hillshade(H: np.ndarray, valid: np.ndarray, cell_m: float, azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    """
    Simple hillshade using gradients on the valid heightmap window.
    """
    H_work = H.copy()
    if not valid.any():
        return np.zeros_like(H_work, dtype=np.float32)
    mean_z = float(np.nanmean(H_work[valid]))
    H_work[~valid] = mean_z

    # Gradients (dz/dx, dz/dy) in meters per pixel
    gy, gx = np.gradient(H_work, cell_m, cell_m)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(gy, -gx)  # aspect measured clockwise from north

    az = math.radians(float(azimuth_deg))
    el = math.radians(float(elevation_deg))
    zen = (math.pi / 2.0) - el

    # Hillshade intensity [0..1]
    shade = (np.cos(zen) * np.cos(slope)) + (np.sin(zen) * np.sin(slope) * np.cos(az - aspect))
    shade = np.clip(shade, 0.0, 1.0)
    return (shade * 255.0).astype(np.float32)

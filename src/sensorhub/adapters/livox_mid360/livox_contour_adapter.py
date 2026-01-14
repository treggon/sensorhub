
# src/sensorhub/adapters/livox_mid360/livox_contour_adapter.py
"""
Livox -> IMU-corrected -> 2.5D heightmap + static contour planes

Concept:
- Read Livox MID-360 frames via sensorhub.manager.latest("livox") (same pattern as voxel adapter).
- Apply a robot base-frame transform (roll/pitch/yaw + translation + scale) before accumulation.
- Accumulate a top-surface heightmap H(x,y) (max occupied z per XY cell) and per-cell hit counts.
- Produce:
  (1) 2D contour image (PNG) with color bands + isolines (marching squares).
  (2) 3D exports for Unity:
      - contour_planes.obj + .mtl: a stack of horizontal colored planes at fixed levels.
      - contours.obj + .mtl: isolines as OBJ line elements (optional; some importers render lines).

Key behaviors:
- Changing the transform clears heightmap immediately (to reflect new pose).
- Optional temporal decay of hit counts/height (to favor recent surface observations).
- Static contour levels are configurable via REST (default spacing: 0.2 m).
- All outputs are robot-centric, relative to the defined base plane (z=0 at tire contact reference).
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
    """
    Accumulates Livox point frames into a robot-centric height field and exports contours.
    """
    def __init__(self,
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
                 **kwargs):
        super().__init__(sensor_id, kind)
        self.log = logging.getLogger(f"sensorhub.adapters.livox_contour.{sensor_id}")
        self._stop = threading.Event()
        # Source Livox adapter id
        self.source_id = source_id
        # Grid geometry
        self.cell = float(cell_size_m)
        self.xy = float(grid_xy_m)
        self.xmin, self.xmax = -self.xy, +self.xy
        self.ymin, self.ymax = -self.xy, +self.xy
        self.nx = int(math.ceil((self.xmax - self.xmin) / self.cell))
        self.ny = int(math.ceil((self.ymax - self.ymin) / self.cell))
        # Z bounds (for clamping & meta)
        self.zmin = float(z_min_m)
        self.zmax = float(z_max_m)
        # Data: heightmap (float32, NaN=unknown) + hits (uint16)
        self.H = np.full((self.nx, self.ny), np.nan, dtype=np.float32)
        self.hits = np.zeros((self.nx, self.ny), dtype=np.uint16)
        # Transform (applied before accumulation)
        self.transform: Dict[str, float] = {
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
            "tx": 0.0, "ty": 0.0, "tz": 0.0, "scale": 1.0
        }
        # Optional IMU cache (passed through by the livox adapter)
        self._imu: Optional[Dict[str, float]] = None
        self.use_gyro_rotation = False  # keep OFF unless accurate timestamps are provided
        self.imu_eps = 0.01
        self.last_imu_ts: Optional[float] = None
        # Publish cadence
        self.publish_period = (1.0 / publish_hz) if (publish_hz and publish_hz > 0.0) else 0.5
        self._last_pub = time.time()
        # Decay controls
        self.decay_enable = bool(decay_enable)
        self.decay_alpha = float(decay_alpha)
        self.decay_period_s = float(decay_period_s)
        self._last_decay = time.time()
        # Contour levels (meters relative to base plane z=0)
        if levels_m is None:
            # default static planes every 0.2 m from -0.5 to +2.0 m
            self.levels = [round(v, 3) for v in np.arange(-0.5, 2.05, 0.2).tolist()]
        else:
            self.levels = [float(v) for v in levels_m]
        # Rendering options
        self.band_cmap = "viridis"
        self.isoline_width_m = 0.02

    # --- lifecycle ---
    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        self._stop.set()
        super().stop()

    # --- transform setter (clears heightmap when changed) ---
    def set_transform(self, t: Dict[str, float]) -> None:
        self.transform = {
            "roll_deg": float(t.get("roll_deg", 0.0)),
            "pitch_deg": float(t.get("pitch_deg", 0.0)),
            "yaw_deg": float(t.get("yaw_deg", 0.0)),
            "tx": float(t.get("tx", 0.0)),
            "ty": float(t.get("ty", 0.0)),
            "tz": float(t.get("tz", 0.0)),
            "scale": float(t.get("scale", 1.0)),
        }
        self.H[:] = np.nan
        self.hits[:] = 0
        self.log.info("Transform updated; heightmap cleared.")

    # --- transforms / IMU ---
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
            x, y, z = x + tx, y + ty, z + tz
            return x, y, z
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
        omega = math.sqrt(gx*gx + gy*gy + gz*gz)
        if omega < self.imu_eps or dt <= 0.0:
            return (1.0, 0.0, 0.0, 0.0)
        hdt = 0.5 * dt
        return (1.0, hdt*gx, hdt*gy, hdt*gz)

    def _apply_quat(self, x: float, y: float, z: float, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        # Small-angle quaternion rotation
        _, qx, qy, qz = q
        vx, vy, vz = x, y, z
        cx = qy*vz - qz*vy
        cy = qz*vx - qx*vz
        cz = qx*vy - qy*vx
        vx += 2.0 * cx
        vy += 2.0 * cy
        vz += 2.0 * cz
        return vx, vy, vz

    # --- accumulation ---
    def _insert_height(self, xr: float, yr: float, zr: float, min_hits_for_update: int = 1) -> None:
        if not (self.xmin <= xr <= self.xmax and self.ymin <= yr <= self.ymax and self.zmin <= zr <= self.zmax):
            return
        ix = int((xr - self.xmin) / self.cell)
        iy = int((yr - self.ymin) / self.cell)
        # Update hit counter and height: keep max observed z as top surface
        self.hits[ix, iy] = min(np.uint16(65535), np.uint16(self.hits[ix, iy] + 1))
        if self.hits[ix, iy] >= min_hits_for_update:
            h_prev = self.H[ix, iy]
            if not math.isfinite(h_prev) or (zr > float(h_prev)):
                self.H[ix, iy] = float(zr)

    # Optional decay of hits & slight relaxation of height toward NaN when few hits remain
    def decay(self, alpha: Optional[float] = None, min_hits_keep: int = 1) -> Dict[str, int]:
        a = float(self.decay_alpha if alpha is None else alpha)
        a = max(0.0, min(1.0, a))
        h = self.hits.astype(np.float32) * a
        self.hits[:] = h.astype(np.uint16)
        # Clear height where hits dropped below threshold
        mask_clear = self.hits < int(min_hits_keep)
        cleared = int(np.count_nonzero(mask_clear))
        if cleared > 0:
            self.H[mask_clear] = np.nan
        kept = int(self.nx*self.ny - cleared)
        return {"kept": kept, "cleared": cleared}

    # --- main loop ---
    def run(self) -> None:
        self.log.info("LivoxContourAdapter run() loop.")
        try:
            while not self._stop.is_set():
                sample = manager.latest(self.source_id)
                if sample and isinstance(sample.data, dict):
                    pts = sample.data.get("points") or []
                    imu = sample.data.get("imu")
                    if imu:
                        self._imu = imu
                    q = (1.0, 0.0, 0.0, 0.0) if not self.use_gyro_rotation else self._imu_rotation_quat()
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
                now = time.time()
                if (now - self._last_pub) >= self.publish_period:
                    self.publish({
                        "sensor_id": self.sensor_id,
                        "status": "running",
                        "timestamp": now,
                        "dims": (self.nx, self.ny),
                        "cell_size_m": self.cell,
                        "bounds": {"xmin": self.xmin, "xmax": self.xmax,
                                   "ymin": self.ymin, "ymax": self.ymax,
                                   "zmin": self.zmin, "zmax": self.zmax},
                        "levels": self.levels,
                    })
                    self._last_pub = now
                # Auto-decay
                try:
                    if self.decay_enable and (time.time() - self._last_decay) >= float(self.decay_period_s):
                        _ = self.decay(self.decay_alpha)
                        self._last_decay = time.time()
                except Exception:
                    pass
                time.sleep(0.01)
        finally:
            self.log.info("LivoxContourAdapter exit.")

    # --- public snapshots ---
    def heightmap_bytes(self) -> bytes:
        # export float32 array; NaN becomes IEEE NaN in raw
        return self.H.astype(np.float32).tobytes(order="C")

    # --- contour computation (marching squares) ---
    def _contours(self, levels: Optional[List[float]] = None) -> Dict[float, List[List[Tuple[float, float, float]]]]:
        """
        Return a dict: level -> list of polylines; each polyline is a list of 3D (x,y,z=level) points.
        Uses simple marching-squares on H.
        """
        H = self.H
        levels = self.levels if levels is None else [float(v) for v in levels]
        out: Dict[float, List[List[Tuple[float, float, float]]]] = {}
        # Precompute corner XY for each cell
        for L in levels:
            polylines: List[List[Tuple[float, float, float]]] = []
            # Marching squares over (nx-1) x (ny-1) cells
            for ix in range(self.nx - 1):
                for iy in range(self.ny - 1):
                    z00 = H[ix, iy]
                    z10 = H[ix+1, iy]
                    z01 = H[ix, iy+1]
                    z11 = H[ix+1, iy+1]
                    if not (math.isfinite(z00) or math.isfinite(z10) or math.isfinite(z01) or math.isfinite(z11)):
                        continue
                    # Build bitmask: 1 if corner >= L
                    b0 = 1 if (math.isfinite(z00) and z00 >= L) else 0
                    b1 = 1 if (math.isfinite(z10) and z10 >= L) else 0
                    b2 = 1 if (math.isfinite(z11) and z11 >= L) else 0
                    b3 = 1 if (math.isfinite(z01) and z01 >= L) else 0
                    code = (b0 << 0) | (b1 << 1) | (b2 << 2) | (b3 << 3)
                    if code == 0 or code == 0xF:
                        continue
                    # Interpolate edges
                    def lerp(a: float, b: float, va: float, vb: float, L: float) -> float:
                        if not (math.isfinite(va) and math.isfinite(vb)):
                            # fall back to midpoint
                            return 0.5 * (a + b)
                        if abs(vb - va) < 1e-9:
                            return 0.5 * (a + b)
                        t = (L - va) / (vb - va)
                        return a + t * (b - a)
                    x0 = self.xmin + ix * self.cell
                    x1 = self.xmin + (ix+1) * self.cell
                    y0 = self.ymin + iy * self.cell
                    y1 = self.ymin + (iy+1) * self.cell
                    edges = []
                    if (b0 != b1):  # bottom edge
                        ex = lerp(x0, x1, z00, z10, L); edges.append((ex, y0))
                    if (b1 != b2):  # right edge
                        ey = lerp(y0, y1, z10, z11, L); edges.append((x1, ey))
                    if (b3 != b2):  # top edge
                        ex = lerp(x0, x1, z01, z11, L); edges.append((ex, y1))
                    if (b0 != b3):  # left edge
                        ey = lerp(y0, y1, z00, z01, L); edges.append((x0, ey))
                    if len(edges) >= 2:
                        polylines.append([(edges[0][0], edges[0][1], L),
                                          (edges[1][0], edges[1][1], L)])
            out[L] = polylines
        return out

# --- REST routes ---

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
            "decay_once": "/livox_contour/decay"
        }
    }

@router.get("/meta")
def contour_meta(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    return {
        "dims": (a.nx, a.ny),
        "cell_size_m": a.cell,
        "bounds": {"xmin": a.xmin, "xmax": a.xmax,
                    "ymin": a.ymin, "ymax": a.ymax,
                    "zmin": a.zmin, "zmax": a.zmax},
        "levels": a.levels,
        "decay": {"enable_decay": a.decay_enable, "decay_alpha": a.decay_alpha, "decay_period_s": a.decay_period_s},
        "transform": a.transform
    }

# --- Levels ---
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

# --- Transform ---
@router.get("/transform")
def get_transform(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    return {"ok": True, "sensor_id": adapter_id, "transform": a.transform}

@router.post("/transform")
def set_transform(adapter_id: str = "livox_contour",
                  roll_deg: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0,
                  tx: float = 0.0, ty: float = 0.0, tz: float = 0.0, scale: float = 1.0):
    a = _get_contour_adapter(adapter_id)
    a.set_transform({
        "roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg,
        "tx": tx, "ty": ty, "tz": tz, "scale": scale
    })
    return {"ok": True, "sensor_id": adapter_id, "transform": a.transform}

# --- Clear & decay ---
@router.post("/clear")
def clear(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    a.H[:] = np.nan
    a.hits[:] = 0
    return {"status": "ok"}

@router.get("/decay/config")
def decay_config(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    return {"enable_decay": bool(a.decay_enable), "decay_alpha": float(a.decay_alpha), "decay_period_s": float(a.decay_period_s)}

@router.post("/decay/config")
def set_decay_config(adapter_id: str = "livox_contour",
                     enable_decay: Optional[int] = None,
                     decay_alpha: Optional[float] = None,
                     decay_period_s: Optional[float] = None):
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

# --- Binary & PNG exports ---
@router.get("/heightmap.raw", response_class=Response)
def heightmap_raw(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    raw = a.heightmap_bytes()
    headers = {"X-Width": str(a.nx), "X-Height": str(a.ny), "X-Cell-M": str(a.cell)}
    return Response(content=raw, media_type="application/octet-stream", headers=headers)

@router.get("/heightmap.png", response_class=Response)
def heightmap_png(adapter_id: str = "livox_contour",
                  scale_mode: str = "auto",
                  gain: float = 1.0,
                  clip_min_m: Optional[float] = None,
                  clip_max_m: Optional[float] = None,
                  cmap: str = "viridis",
                  draw_grid: int = 0,
                  tick_m: float = 1.0,
                  mark_center: int = 1,
                  invert_y: int = 0,
                  downscale: int = 2):
    a = _get_contour_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")
    H = a.H.copy()
    # Replace NaN with zmin for visualization baseline
    H_vis = np.where(np.isfinite(H), H, a.zmin).astype(np.float32)
    if invert_y:
        H_vis = np.flipud(H_vis)
    # Scaling
    if clip_min_m is not None or clip_max_m is not None:
        lo = a.zmin if clip_min_m is None else float(clip_min_m)
        hi = a.zmax if clip_max_m is None else float(clip_max_m)
        H_vis = np.clip(H_vis, lo, hi)
    arr = H_vis
    if scale_mode == "auto":
        p5, p95 = np.percentile(arr, 5), np.percentile(arr, 95)
        arr = (arr - p5) * (255.0 / max(1e-6, p95 - p5))
        arr = np.clip(arr, 0.0, 255.0)
        img = Image.fromarray(arr.astype(np.uint8), mode="L")
    elif scale_mode == "equalize":
        arr = np.clip((arr - a.zmin) * (255.0 / max(1e-3, (a.zmax - a.zmin))) * gain, 0.0, 255.0).astype(np.uint8)
        img = Image.fromarray(arr, mode="L")
        try:
            img = ImageOps.equalize(img)
        except Exception:
            pass
    else:
        arr = np.clip((arr - a.zmin) * (255.0 / max(1e-3, (a.zmax - a.zmin))) * gain, 0.0, 255.0)
        img = Image.fromarray(arr.astype(np.uint8), mode="L")
    # colormap
    img = _apply_cmap(img, cmap)
    # overlays
    draw = ImageDraw.Draw(img)
    w, h = img.size
    if draw_grid:
        spacing_px = int(round(tick_m / a.cell))
        if spacing_px >= 1:
            col = (200, 200, 200)
            for x in range(0, w, spacing_px):
                draw.line([(x, 0), (x, h-1)], fill=col, width=1)
            for y in range(0, h, spacing_px):
                draw.line([(0, y), (w-1, y)], fill=col, width=1)
    if mark_center:
        cx = int(round((0.0 - a.xmin) / a.cell))
        cy = int(round((0.0 - a.ymin) / a.cell))
        if invert_y:
            cy = (a.ny - 1) - cy
        cx = max(0, min(w-1, cx))
        cy = max(0, min(h-1, cy))
        draw.line([(cx-6, cy), (cx+6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy-6), (cx, cy+6)], fill=(255, 0, 0), width=2)
    if downscale and int(downscale) > 1:
        factor = int(downscale)
        img = img.resize((max(1, w//factor), max(1, h//factor)), resample=Image.BILINEAR)
    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {"X-Width": str(img.size[0]), "X-Height": str(img.size[1]), "X-Cell-M": str(a.cell)}
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)

@router.get("/contours.png", response_class=Response)
def contours_png(adapter_id: str = "livox_contour",
                 cmap: str = "viridis",
                 line_color: str = "white",
                 line_alpha: float = 0.9,
                 downscale: int = 2):
    a = _get_contour_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")
    # Base band image
    base = heightmap_png(adapter_id=adapter_id, cmap=cmap, downscale=1)
    try:
        from io import BytesIO
        img = Image.open(BytesIO(base.body))
    except Exception as e:
        raise HTTPException(500, f"failed to build base image: {e}")
    draw = ImageDraw.Draw(img)
    # Convert color string to RGB
    col = (255, 255, 255) if line_color.lower() == "white" else (255, 0, 0)
    # Draw isolines
    contours = a._contours(a.levels)
    for L, lines in contours.items():
        rgba = (*col, int(max(0, min(255, round(255.0 * line_alpha)))))
        for line in lines:
            pts_px = []
            for (x, y, _z) in line:
                px = int(round((x - a.xmin) / a.cell))
                py = int(round((y - a.ymin) / a.cell))
                pts_px.append((px, py))
            if len(pts_px) >= 2:
                draw.line(pts_px, fill=rgba, width=1)
    if downscale and int(downscale) > 1:
        factor = int(downscale)
        img = img.resize((img.size[0]//factor, img.size[1]//factor), resample=Image.BILINEAR)
    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {"X-Width": str(img.size[0]), "X-Height": str(img.size[1]), "X-Cell-M": str(a.cell)}
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)

# --- OBJ exports ---
@router.get("/contour_planes.obj", response_class=Response)
def contour_planes_obj(adapter_id: str = "livox_contour"):
    a = _get_contour_adapter(adapter_id)
    # Simple OBJ + MTL idea: one quad per level with a per-level material (fast in Unity)
    mtl_name = "contour_planes.mtl"
    obj_lines = []
    mtl_lines = ["# contour planes materials\n"]
    def level_color(k: int, n: int) -> Tuple[int, int, int]:
        t = k / max(1, n-1)
        r = int(68 + 187*t)
        g = int(1 + 255*t)
        b = int(84 + 140*t)
        r = max(0, min(255, r)); g = max(0, min(255, g)); b = max(0, min(255, b))
        return r, g, b
    v_idx = 1
    obj_lines.append(f"mtllib {mtl_name}\n")
    for i, L in enumerate(a.levels):
        r, g, b = level_color(i, len(a.levels))
        mtl = f"level_{i}"
        mtl_lines.append(f"newmtl {mtl}\nKd {r/255.0:.4f} {g/255.0:.4f} {b/255.0:.4f}\n\n")
        v = [
            (a.xmin, a.ymin, L),
            (a.xmax, a.ymin, L),
            (a.xmax, a.ymax, L),
            (a.xmin, a.ymax, L)
        ]
        for (x,y,z) in v:
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
    contours = a._contours(a.levels)
    obj_lines = []
    obj_lines.append("# contour isolines\n")
    obj_lines.append("o contours\n")
    v_idx = 1
    for i, L in enumerate(a.levels):
        obj_lines.append(f"g level_{i}\n")
        for poly in contours.get(L, []):
            idxs = []
            for (x,y,z) in poly:
                obj_lines.append(f"v {x:.5f} {y:.5f} {z:.5f}\n")
                idxs.append(v_idx)
                v_idx += 1
            if len(idxs) >= 2:
                obj_lines.append("l " + " ".join(str(k) for k in idxs) + "\n")
    obj = "".join(obj_lines)
    headers = {"X-Levels": json.dumps(a.levels)}
    return Response(content=obj.encode("ascii"), media_type="text/plain", headers=headers)

# --- helpers ---
def _apply_cmap(img: "Image.Image", cmap: str) -> "Image.Image":
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
            r = int(68 + 187*t); g = int(1 + 255*t); b = int(84 + 140*t)
            r = max(0, min(255, r)); g = max(0, min(255, g)); b = max(0, min(255, b))
            lut += [r, g, b]
    if lut:
        img = img.convert("P"); img.putpalette(lut); img = img.convert("RGB")
    else:
        img = img.convert("RGB")
    return img


# src/sensorhub/adapters/livox_mid360/livox_voxel_adapter.py
"""
Livox -> IMU-corrected -> 3D voxel grid accumulator (8-bit per voxel)

- 0.0125 m voxel size (higher resolution by default)
- ±20 m XY, Z=[-2, +8] m (configurable)
- Occupancy strength projection (top-down)
- BINVOX v2 export (stores 0..255 voxel values, not only binary)
- WebSocket stream for top-down tiles
- Traversability probe endpoint

NEW:
- Radial slope analysis: 'first few voxels' per direction for cliff detection
- /livox_voxel/topdown_filtered.png: colorized radial overlay (green/yellow/orange/red)
- /livox_voxel/traverse/check: returns both OK (traverse) and OK_cliff (no cliff),
  plus per-direction slope angles and classes.
"""
import math, time, threading, logging, json, struct, asyncio
from typing import Dict, Tuple, Optional, List
import numpy as np
from fastapi import APIRouter, HTTPException, Response, WebSocket, WebSocketDisconnect
from sensorhub.core.sensor_base import AbstractSensorAdapter
from sensorhub.core.sensor_manager import manager

try:
    from PIL import Image
    from PIL import ImageDraw
    PIL_OK = True
except Exception:
    PIL_OK = False

router = APIRouter(prefix="/livox_voxel", tags=["livox_voxel"])


# -------------------------------
# Adapter
# -------------------------------
class LivoxVoxelAdapter(AbstractSensorAdapter):
    """
    Accumulates Livox point frames into a 3D voxel grid centered on robot.
    8-bit voxel code:
      bit7: occupied (1=occupied)
      bits6..5: class (00=unknown, 01=surface/ground, 10=obstacle, 11=reserved)
      bits4..0: saturating strength (hit count 0..31) or intensity bucket
    """
    def __init__(self,
                 sensor_id: str,
                 kind: str = "voxelgrid",
                 source_id: str = "livox",
                 voxel_size_m: float = 0.0125,   # higher resolution default
                 grid_xy_m: float = 20.0,
                 grid_z_m: Tuple[float, float] = (-2.0, 8.0),
                 chunk_size: int = 32,
                 imu_threshold_radps: float = 0.01,
                 min_hits_for_occupied: int = 3,
                 publish_hz: Optional[float] = 2.0,
                 climb_limit_deg: float = 45.0,
                 **kwargs):
        super().__init__(sensor_id, kind)
        self.log = logging.getLogger(f"sensorhub.adapters.livox_voxel.{sensor_id}")
        self._stop = threading.Event()

        # Source Livox adapter id
        self.source_id = source_id

        # Grid geometry
        self.voxel = float(voxel_size_m)
        self.xy = float(grid_xy_m)
        self.zmin, self.zmax = float(grid_z_m[0]), float(grid_z_m[1])
        self.nx = int(math.ceil((+self.xy - (-self.xy)) / self.voxel))  # span 2*xy
        self.ny = int(math.ceil((+self.xy - (-self.xy)) / self.voxel))
        self.nz = int(math.ceil((self.zmax - self.zmin) / self.voxel))
        self.xmin, self.xmax = -self.xy, +self.xy
        self.ymin, self.ymax = -self.xy, +self.xy

        # Data
        self.grid = np.zeros((self.nx, self.ny, self.nz), dtype=np.uint8)

        # IMU / motion compensation
        self.imu_eps = float(imu_threshold_radps)
        self._imu: Optional[Dict[str, float]] = None

        # Occupancy controls
        self.min_hits_for_occupied = int(min_hits_for_occupied)

        # Publish cadence
        self.publish_period = (1.0 / publish_hz) if (publish_hz and publish_hz > 0.0) else 0.5
        self._last_pub = time.time()

        # Top-down projection cache (full volume)
        self._topdown = np.zeros((self.nx, self.ny), dtype=np.uint8)
        self._topdown_dirty = True

        # Slope meta
        self.climb_limit_deg = float(climb_limit_deg)

    # --- lifecycle ---
    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        self._stop.set()
        super().stop()

    # --- transforms / IMU ---
    def _apply_transform(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        t = getattr(self, "transform", None)
        if t is None:
            return x, y, z
        try:
            sx = getattr(t, "scale", 1.0) if hasattr(t, "scale") else float(t.get("scale", 1.0))
            rx = math.radians(getattr(t, "roll_deg", 0.0) if hasattr(t, "roll_deg") else float(t.get("roll_deg", 0.0)))
            ry = math.radians(getattr(t, "pitch_deg", 0.0) if hasattr(t, "pitch_deg") else float(t.get("pitch_deg", 0.0)))
            rz = math.radians(getattr(t, "yaw_deg", 0.0) if hasattr(t, "yaw_deg") else float(t.get("yaw_deg", 0.0)))
            tx = getattr(t, "tx", 0.0) if hasattr(t, "tx") else float(t.get("tx", 0.0))
            ty = getattr(t, "ty", 0.0) if hasattr(t, "ty") else float(t.get("ty", 0.0))
            tz = getattr(t, "tz", 0.0) if hasattr(t, "tz") else float(t.get("tz", 0.0))

            x, y, z = sx * x, sx * y, sx * z
            cz, sz = math.cos(rz), math.sin(rz)
            cy, sy = math.cos(ry), math.sin(ry)
            cx, sx_ = math.cos(rx), math.sin(rx)
            # Rz
            x, y = (cz * x - sz * y), (sz * x + cz * y)
            # Ry
            x, z = (cy * x + sy * z), (-sy * x + cy * z)
            # Rx
            y, z = (cx * y - sx_ * z), (sx_ * y + cx * z)
            x, y, z = x + tx, y + ty, z + tz
            return x, y, z
        except Exception:
            return x, y, z

    def _imu_rotation_quat(self) -> Tuple[float, float, float, float]:
        if not self._imu:
            return (1.0, 0.0, 0.0, 0.0)
        gx = float(self._imu.get("gx_radps", 0.0))
        gy = float(self._imu.get("gy_radps", 0.0))
        gz = float(self._imu.get("gz_radps", 0.0))
        omega = math.sqrt(gx*gx + gy*gy + gz*gz)
        if omega < self.imu_eps:
            return (1.0, 0.0, 0.0, 0.0)
        dt = 0.01
        half = 0.5 * dt
        return (1.0, half*gx, half*gy, half*gz)

    def _apply_quat(self, x: float, y: float, z: float, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        w, qx, qy, qz = q
        vx, vy, vz = x, y, z
        cx = qy*vz - qz*vy
        cy = qz*vx - qx*vz
        cz = qx*vy - qy*vx
        vx += 2.0 * cx
        vy += 2.0 * cy
        vz += 2.0 * cz
        return vx, vy, vz

    # --- insert & projection ---
    def _insert_voxel(self, xr: float, yr: float, zr: float, intensity: int) -> None:
        if not (self.xmin <= xr <= self.xmax and self.ymin <= yr <= self.ymax and self.zmin <= zr <= self.zmax):
            return
        ix = int((xr - self.xmin) / self.voxel)
        iy = int((yr - self.ymin) / self.voxel)
        iz = int((zr - self.zmin) / self.voxel)
        v = self.grid[ix, iy, iz]
        hits = (v & 0x1F) + 1
        v = (0x80) | min(31, hits)   # occupied + saturating strength
        if zr > 0.3:
            v = (v & 0x9F) | (0b10 << 5)  # obstacle
        else:
            v = (v & 0x9F) | (0b01 << 5)  # surface/ground
        self.grid[ix, iy, iz] = v
        self._topdown_dirty = True

    def _update_topdown(self) -> None:
        g = self.grid
        occ = (g & 0x80) > 0
        strength = (g & 0x1F)
        max_s = np.where(occ.any(axis=2), strength.max(axis=2), 0)
        self._topdown[:] = max_s.astype(np.uint8)
        self._topdown_dirty = False

    # --- window helpers ---
    def _clamped_window_indices(self,
                                x_min: float, x_max: float,
                                y_min: float, y_max: float,
                                z_min: float, z_max: float) -> Tuple[int,int,int,int,int,int]:
        x_min = max(self.xmin, x_min); x_max = min(self.xmax, x_max)
        y_min = max(self.ymin, y_min); y_max = min(self.ymax, y_max)
        z_min = max(self.zmin, z_min); z_max = min(self.zmax, z_max)
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
            return 0, -1, 0, -1, 0, -1  # empty
        ix0 = int((x_min - self.xmin) / self.voxel)
        ix1 = int((x_max - self.xmin) / self.voxel)
        iy0 = int((y_min - self.ymin) / self.voxel)
        iy1 = int((y_max - self.ymin) / self.voxel)
        iz0 = int((z_min - self.zmin) / self.voxel)
        iz1 = int((z_max - self.zmin) / self.voxel)
        ix1 = min(ix1, self.nx - 1)
        iy1 = min(iy1, self.ny - 1)
        iz1 = min(iz1, self.nz - 1)
        return ix0, ix1, iy0, iy1, iz0, iz1

    def _topdown_from_window(self,
                             x_min: float, x_max: float,
                             y_min: float, y_max: float,
                             z_min: float, z_max: float) -> np.ndarray:
        ix0, ix1, iy0, iy1, iz0, iz1 = self._clamped_window_indices(x_min, x_max, y_min, y_max, z_min, z_max)
        if ix1 < ix0 or iy1 < iy0 or iz1 < iz0:
            return np.zeros((1, 1), dtype=np.uint8)
        g = self.grid[ix0:ix1+1, iy0:iy1+1, iz0:iz1+1]
        occ = (g & 0x80) > 0
        strength = (g & 0x1F)
        proj = np.where(occ.any(axis=2), strength.max(axis=2), 0).astype(np.uint8)
        return proj

    def _height_at_xy(self, ix: int, iy: int, iz0: int, iz1: int) -> Optional[float]:
        """Max occupied Z at (ix, iy) within Z slice [iz0..iz1]."""
        if ix < 0 or iy < 0 or ix >= self.nx or iy >= self.ny:
            return None
        col = self.grid[ix, iy, max(0, iz0):min(self.nz - 1, iz1) + 1]
        occ = (col & 0x80) > 0
        if not occ.any():
            return None
        iz_local = int(np.where(occ)[0].max())
        iz_abs = max(0, iz0) + iz_local
        return self.zmin + iz_abs * self.voxel

    def _heightmap_from_window(self,
                               x_min: float, x_max: float,
                               y_min: float, y_max: float,
                               z_min: float, z_max: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (H, xs, ys):
           H is (nxw, nyw) with max occupied z per column (np.nan if none).
           xs, ys are 1D arrays of world coords at each column/row center.
        """
        ix0, ix1, iy0, iy1, iz0, iz1 = self._clamped_window_indices(x_min, x_max, y_min, y_max, z_min, z_max)
        if ix1 < ix0 or iy1 < iy0 or iz1 < iz0:
            return np.full((1,1), np.nan, dtype=np.float32), np.array([0.0]), np.array([0.0])

        g = self.grid[ix0:ix1+1, iy0:iy1+1, iz0:iz1+1]
        occ = (g & 0x80) > 0
        H = np.full((g.shape[0], g.shape[1]), np.nan, dtype=np.float32)
        any_occ = occ.any(axis=2)
        where_any = np.where(any_occ)
        if where_any[0].size > 0:
            # highest occupied along z
            occ_rev = occ[:, :, ::-1]
            iz_rev = np.argmax(occ_rev, axis=2)  # index from top of slice
            iz_top = occ.shape[2] - 1 - iz_rev
            H[any_occ] = (self.zmin + (iz0 + iz_top[any_occ]) * self.voxel).astype(np.float32)

        xs = self.xmin + (np.arange(ix0, ix1+1) * self.voxel)
        ys = self.ymin + (np.arange(iy0, iy1+1) * self.voxel)
        return H, xs, ys

    # --- radial slope analysis (initial voxels along directions) ---
    def _radial_slopes(self,
                       radial_bins: int,
                       sample_m: float,
                       first_n: int,
                       x0: float, y0: float,
                       iz0: int, iz1: int,
                       forward_only: bool) -> List[Dict[str, float]]:
        """
        For each azimuth bin, sample the first N valid voxels within 'sample_m' along that direction
        starting at (x0, y0) in world coords, and compute an initial slope angle (deg).
        Returns list of {theta_deg, pitch_deg, dr_m, dz_m, samples}.
        """
        out: List[Dict[str, float]] = []
        if radial_bins <= 0:
            return out
        # azimuth range (relative to +X)
        if forward_only:
            theta_start = -math.pi/2
            theta_end   = +math.pi/2
        else:
            theta_start = -math.pi
            theta_end   = +math.pi

        for k in range(radial_bins):
            # center angle of bin
            t = theta_start + (k + 0.5) * ((theta_end - theta_start) / radial_bins)
            # march along ray
            hs: List[float] = []
            rs: List[float] = []
            steps = max(1, int(round(sample_m / self.voxel)))
            for j in range(1, steps + 1):
                r = j * self.voxel
                xw = x0 + r * math.cos(t)
                yw = y0 + r * math.sin(t)
                ix = int((xw - self.xmin) / self.voxel)
                iy = int((yw - self.ymin) / self.voxel)
                h = self._height_at_xy(ix, iy, iz0, iz1)
                if h is not None:
                    hs.append(h)
                    rs.append(r)
                    if len(hs) >= first_n:
                        break
            if len(hs) >= 2:
                dz = hs[-1] - hs[0]
                dr = rs[-1] - rs[0]
                angle = math.degrees(math.atan2(dz, max(dr, 1e-6)))
                out.append({"theta_deg": math.degrees(t), "pitch_deg": angle, "dr_m": dr, "dz_m": dz, "samples": len(hs)})
            else:
                out.append({"theta_deg": math.degrees(t), "pitch_deg": float("nan"), "dr_m": 0.0, "dz_m": 0.0, "samples": len(hs)})
        return out

    # --- main loop ---
    def run(self) -> None:
        self.log.info("LivoxVoxelAdapter run() loop.")
        try:
            while not self._stop.is_set():
                sample = manager.latest(self.source_id)
                if sample and isinstance(sample.data, dict):
                    pts = sample.data.get("points") or []
                    imu = sample.data.get("imu")
                    if imu:
                        self._imu = imu
                    q = self._imu_rotation_quat()
                    for p in pts:
                        if len(p) < 2:
                            continue
                        if len(p) == 2:
                            x, y = float(p[0]), float(p[1]); z, i = 0.0, 0
                        elif len(p) == 3:
                            x, y, z = float(p[0]), float(p[1]), float(p[2]); i = 0
                        else:
                            x, y, z, i = float(p[0]), float(p[1]), float(p[2]), int(p[3])
                        x, y, z = self._apply_quat(x, y, z, q)
                        xr, yr, zr = self._apply_transform(x, y, z)
                        self._insert_voxel(xr, yr, zr, i)

                now = time.time()
                if (now - self._last_pub) >= self.publish_period:
                    self._update_topdown()
                    self.publish({
                        "sensor_id": self.sensor_id,
                        "status": "running",
                        "timestamp": now,
                        "dims": (self.nx, self.ny, self.nz),
                        "voxel_size_m": self.voxel,
                        "bounds": {"xmin": self.xmin, "xmax": self.xmax,
                                   "ymin": self.ymin, "ymax": self.ymax,
                                   "zmin": self.zmin, "zmax": self.zmax},
                        "climb_limit_deg": self.climb_limit_deg
                    })
                    self._last_pub = now
                time.sleep(0.01)
        finally:
            self.log.info("LivoxVoxelAdapter exit.")

    # --- public snapshots ---
    def topdown_bytes(self) -> bytes:
        if self._topdown_dirty:
            self._update_topdown()
        return self._topdown.tobytes(order="C")


# -------------------------------
# REST routes
# -------------------------------
def _get_voxel_adapter(adapter_id: str = "livox_voxel") -> LivoxVoxelAdapter:
    a = manager.get_adapter(adapter_id)
    if not a or not isinstance(a, LivoxVoxelAdapter):
        raise HTTPException(404, "voxel adapter not found")
    return a


@router.get("/routes")
def list_routes(adapter_id: str = "livox_voxel"):
    return {
        "adapter_id": adapter_id,
        "routes": {
            "meta": "/livox_voxel/meta",
            "binvox": "/livox_voxel/grid.binvox",
            "topdown_raw": "/livox_voxel/topdown.raw",
            "topdown_png": "/livox_voxel/topdown.png",
            "topdown_filtered": "/livox_voxel/topdown_filtered.png",
            "ws_topdown": "/livox_voxel/ws/topdown",
            "traverse_check": "/livox_voxel/traverse/check"
        }
    }


@router.get("/meta")
def voxel_meta(adapter_id: str = "livox_voxel"):
    a = _get_voxel_adapter(adapter_id)
    return {
        "dims": (a.nx, a.ny, a.nz),
        "voxel_size_m": a.voxel,
        "bounds": {"xmin": a.xmin, "xmax": a.xmax,
                   "ymin": a.ymin, "ymax": a.ymax,
                   "zmin": a.zmin, "zmax": a.zmax},
        "code_semantics": "bit7=occupied; bits6..5=class; bits4..0=strength(0..31)",
        "climb_limit_deg": a.climb_limit_deg
    }


@router.get("/grid.binvox", response_class=Response)
def grid_binvox(adapter_id: str = "livox_voxel", version: int = 2):
    a = _get_voxel_adapter(adapter_id)
    g = a.grid
    header = (f"#binvox {version}\n"
              f"dim {a.nx} {a.ny} {a.nz}\n"
              f"translate 0 0 0\nscale 1.0\ndata\n").encode("ascii")
    payload = bytearray()
    flat = g.flatten(order="C")
    i = 0
    while i < flat.size:
        val = int(flat[i])
        run = 1
        j = i + 1
        while j < flat.size and int(flat[j]) == val and run < 255:
            run += 1; j += 1
        payload.append(val & 0xFF)
        payload.append(run & 0xFF)
        i = j
    return Response(content=header + bytes(payload), media_type="application/octet-stream")


@router.get("/topdown.raw", response_class=Response)
def topdown_raw(adapter_id: str = "livox_voxel"):
    a = _get_voxel_adapter(adapter_id)
    raw = a.topdown_bytes()
    headers = {"X-Width": str(a.nx), "X-Height": str(a.ny), "X-Voxel-M": str(a.voxel)}
    return Response(content=raw, media_type="application/octet-stream", headers=headers)


@router.get("/topdown.png", response_class=Response)
def topdown_png(
    adapter_id: str = "livox_voxel",
    scale_mode: str = "auto",
    gain: float = 8.0,
    clip_min: int = 0,
    clip_max: int = 31,
    cmap: str = "gray",
    draw_grid: int = 0,
    tick_m: float = 1.0,
    mark_center: int = 1,
    invert_y: int = 0,
    crop: int = 0,
    crop_radius_m: float = 10.0,
    downscale: int = 2
):
    a = _get_voxel_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")

    if a._topdown_dirty:
        a._update_topdown()
    td = a._topdown

    arr = td if not invert_y else np.flipud(td)
    arr = arr.astype(np.float32)
    arr = np.clip(arr, float(clip_min), float(clip_max))

    if scale_mode == "auto":
        p5, p95 = np.percentile(arr, 5), np.percentile(arr, 95)
        arr = (arr - p5) * (255.0 / max(1e-6, p95 - p5))
        arr = np.clip(arr, 0.0, 255.0)
    elif scale_mode == "equalize":
        arr = np.clip(arr * gain * (255.0 / 31.0), 0.0, 255.0).astype(np.uint8)
        img = Image.fromarray(arr, mode="L")
        try:
            from PIL import ImageOps
            img = ImageOps.equalize(img)
        except Exception:
            pass
    else:
        arr = np.clip(arr * gain * (255.0 / 31.0), 0.0, 255.0)

    if scale_mode != "equalize":
        img = Image.fromarray(arr.astype(np.uint8), mode="L")

    # colormap
    def _apply_cmap(_img: "Image.Image") -> "Image.Image":
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
            _img = _img.convert("P"); _img.putpalette(lut); _img = _img.convert("RGB")
        else:
            _img = _img.convert("RGB")
        return _img

    img = _apply_cmap(img)

    # overlays
    draw = ImageDraw.Draw(img)
    w, h = img.size
    if draw_grid:
        spacing_px = int(round(tick_m / a.voxel))
        if spacing_px >= 1:
            col = (200, 200, 200) if cmap == "gray" else (255, 255, 255)
            for x in range(0, w, spacing_px):
                draw.line([(x, 0), (x, h - 1)], fill=col, width=1)
            for y in range(0, h, spacing_px):
                draw.line([(0, y), (w - 1, y)], fill=col, width=1)

    if mark_center:
        cx_full = int(round((0.0 - a.xmin) / a.voxel))
        cy_full = int(round((0.0 - a.ymin) / a.voxel))
        if invert_y:
            cy_full = (arr.shape[0] - 1) - cy_full
        cx = max(0, min(w - 1, cx_full))
        cy = max(0, min(h - 1, cy_full))
        draw.line([(cx - 6, cy), (cx + 6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy - 6), (cx, cy + 6)], fill=(255, 0, 0), width=2)

    if downscale and int(downscale) > 1:
        factor = int(downscale)
        nw = max(1, w // factor); nh = max(1, h // factor)
        img = img.resize((nw, nh), resample=Image.BILINEAR)
        w, h = nw, nh

    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {"X-Width": str(w), "X-Height": str(h), "X-Voxel-M": str(a.voxel)}
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)


# --- Filtered local topdown with forward-only & window toggles + colorized radial overlay ---
@router.get("/topdown_filtered.png", response_class=Response)
def topdown_filtered_png(
    adapter_id: str = "livox_voxel",
    radius_m: float = 1.0,
    z_center_m: float = 0.0,
    z_half_thickness_m: float = 1.0,
    forward_only: int = 1,
    window: int = 1,      # XY window toggle
    z_window: int = 1,    # Z window toggle
    # visualization
    scale_mode: str = "auto",
    gain: float = 8.0,
    clip_min: int = 0,
    clip_max: int = 31,
    cmap: str = "gray",
    draw_grid: int = 1,
    tick_m: float = 0.25,
    mark_center: int = 1,
    invert_y: int = 0,
    downscale: int = 1,
    upscale: int = 1,
    upscale_mode: str = "nearest",
    # radial overlay (cliff visualization)
    draw_radials: int = 1,
    radial_bins: int = 16,
    sample_m: float = 1.0,
    first_n: int = 3,
    flat_thresh_deg: float = 10.0,
    moderate_thresh_deg: float = 30.0,
    cliff_limit_deg: float = 45.0
):
    a = _get_voxel_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")

    r = max(0.01, float(radius_m))
    zc = float(z_center_m)
    zh = max(0.01, float(z_half_thickness_m))

    # XY window
    if int(window) == 1:
        if int(forward_only) == 1:
            x0, x1 = 0.0, +r
        else:
            x0, x1 = -r, +r
        y0, y1 = -r, +r
    else:
        x0, x1 = a.xmin, a.xmax
        y0, y1 = a.ymin, a.ymax

    # Z window
    if int(z_window) == 1:
        z0, z1 = zc - zh, zc + zh
    else:
        z0, z1 = a.zmin, a.zmax

    # Project slice for base grayscale
    proj = a._topdown_from_window(x0, x1, y0, y1, z0, z1) if (window or z_window) else a._topdown
    if invert_y:
        proj = np.flipud(proj)

    arr = proj.astype(np.float32)
    arr = np.clip(arr, float(clip_min), float(clip_max))

    # scale to image
    if scale_mode == "auto":
        p5, p95 = np.percentile(arr, 5), np.percentile(arr, 95)
        arr = (arr - p5) * (255.0 / max(1e-6, p95 - p5))
        arr = np.clip(arr, 0.0, 255.0)
        img = Image.fromarray(arr.astype(np.uint8), mode="L")
    elif scale_mode == "equalize":
        arr = np.clip(arr * gain * (255.0 / 31.0), 0.0, 255.0).astype(np.uint8)
        img = Image.fromarray(arr, mode="L")
        try:
            from PIL import ImageOps
            img = ImageOps.equalize(img)
        except Exception:
            pass
    else:
        arr = np.clip(arr * gain * (255.0 / 31.0), 0.0, 255.0)
        img = Image.fromarray(arr.astype(np.uint8), mode="L")

    # colormap -> RGB
    def _apply_cmap(_img: "Image.Image") -> "Image.Image":
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
            _img = _img.convert("P"); _img.putpalette(lut); _img = _img.convert("RGB")
        else:
            _img = _img.convert("RGB")
        return _img

    img = _apply_cmap(img)
    draw = ImageDraw.Draw(img)
    h_px, w_px = proj.shape[0], proj.shape[1]  # (height=x, width=y)
    w_img, h_img = img.size

    # grid overlay
    if draw_grid:
        spacing_px = max(1, int(round(tick_m / a.voxel)))
        col = (200, 200, 200) if cmap == "gray" else (255, 255, 255)
        for x in range(0, w_img, spacing_px):
            draw.line([(x, 0), (x, h_img - 1)], fill=col, width=1)
        for y in range(0, h_img, spacing_px):
            draw.line([(0, y), (w_img - 1, y)], fill=col, width=1)

    # crosshair at sensor center
    px_x = int(round((0.0 - x0) / a.voxel))
    px_y = int(round((0.0 - y0) / a.voxel))
    px_x = max(0, min(h_px - 1, px_x))
    px_y = max(0, min(w_px - 1, px_y))
    if mark_center:
        cx = max(0, min(w_img - 1, px_y))
        cy = max(0, min(h_img - 1, px_x))
        draw.line([(cx - 6, cy), (cx + 6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy - 6), (cx, cy + 6)], fill=(255, 0, 0), width=2)

    # --- radial slope overlay ---
    radial_summary = []
    if int(draw_radials) == 1:
        # convert Z slice to indices for sampling
        ix0, ix1, iy0, iy1, iz0_idx, iz1_idx = a._clamped_window_indices(x0, x1, y0, y1, z0, z1)
        # world center:
        x_center, y_center = 0.0, 0.0
        # analyze
        sectors = a._radial_slopes(
            radial_bins=int(radial_bins),
            sample_m=float(sample_m),
            first_n=int(first_n),
            x0=x_center, y0=y_center,
            iz0=iz0_idx, iz1=iz1_idx,
            forward_only=bool(int(forward_only))
        )
        radial_summary = sectors
        # draw rays with color based on thresholds
        color_flat   = (0, 200, 0)     # green
        color_mod    = (255, 215, 0)   # yellow
        color_steep  = (255, 140, 0)   # orange
        color_cliff  = (255, 0, 0)     # red
        max_len_px   = max(1, int(round(float(sample_m) / a.voxel)))

        for s in sectors:
            ang = s["theta_deg"]
            pitch = s["pitch_deg"]
            # pick color
            if not math.isfinite(pitch):
                col = (128, 128, 128)  # no data
            elif abs(pitch) >= float(cliff_limit_deg):
                col = color_cliff
            elif abs(pitch) >= float(moderate_thresh_deg):
                col = color_steep
            elif abs(pitch) >= float(flat_thresh_deg):
                col = color_mod
            else:
                col = color_flat
            # ray endpoint in pixel space (image columns = +Y, rows = +X)
            dx_px = int(round(max_len_px * math.sin(math.radians(ang))))
            dy_px = int(round(max_len_px * math.cos(math.radians(ang))))
            x1p = max(0, min(w_img - 1, px_y + dx_px))
            y1p = max(0, min(h_img - 1, px_x - dy_px))
            draw.line([(cx, cy), (x1p, y1p)], fill=col, width=2)

    # downscale/upscale for UI
    if downscale and int(downscale) > 1:
        factor = int(downscale)
        img = img.resize((max(1, img.size[0]//factor), max(1, img.size[1]//factor)), resample=Image.BILINEAR)
    if upscale and int(upscale) > 1:
        factor = int(upscale)
        res = Image.BILINEAR if upscale_mode.lower() == "bilinear" else Image.NEAREST
        img = img.resize((img.size[0]*factor, img.size[1]*factor), resample=res)

    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {
        "X-Width": str(img.size[0]),
        "X-Height": str(img.size[1]),
        "X-Voxel-M": str(a.voxel),
        "X-Window": json.dumps({"x": [round(x0, 3), round(x1, 3)],
                                "y": [round(y0, 3), round(y1, 3)],
                                "z": [round(z0, 3), round(z1, 3)],
                                "forward_only": bool(int(forward_only)),
                                "window": bool(int(window)),
                                "z_window": bool(int(z_window))}),
        "X-Radials": json.dumps({"bins": int(radial_bins),
                                 "sample_m": float(sample_m),
                                 "first_n": int(first_n),
                                 "flat_deg": float(flat_thresh_deg),
                                 "moderate_deg": float(moderate_thresh_deg),
                                 "cliff_deg": float(cliff_limit_deg)})
    }
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)


# -------------------------------
# WebSocket: push top-down tiles
# -------------------------------
@router.websocket("/ws/topdown")
async def ws_topdown(ws: WebSocket, adapter_id: str = "livox_voxel", period_ms: int = 500):
    await ws.accept()
    try:
        a = _get_voxel_adapter(adapter_id)
        while True:
            nx, ny = a.nx, a.ny
            voxel_mm = int(a.voxel * 1000.0)
            payload = a.topdown_bytes()
            header = struct.pack("<III", nx, ny, voxel_mm)
            await ws.send_bytes(header + payload)
            await asyncio.sleep(max(0.05, float(period_ms) / 1000.0))
    except WebSocketDisconnect:
        return
    except Exception:
        try:
            await ws.close(code=1011)
        except Exception:
            pass


# -------------------------------
# Traversability probe (REST)
# -------------------------------
def _column_max_z(grid: np.ndarray, ix: int, iy: int,
                  zmin: float, voxel: float,
                  iz0: Optional[int] = None, iz1: Optional[int] = None) -> Optional[float]:
    col = grid[ix, iy, :]
    if iz0 is not None or iz1 is not None:
        nz = col.shape[0]
        s0 = 0 if iz0 is None else max(0, min(nz - 1, iz0))
        s1 = (nz - 1) if iz1 is None else max(0, min(nz - 1, iz1))
        if s1 < s0:
            s0, s1 = s1, s0
        col = col[s0:s1 + 1]
        offset = s0
    else:
        offset = 0
    occ = (col & 0x80) > 0
    if not occ.any():
        return None
    iz = int(np.where(occ)[0].max()) + offset
    return zmin + iz * voxel


@router.get("/traverse/check", response_class=Response)
def traverse_check(adapter_id: str = "livox_voxel",
                   ahead_m: float = 1.0,
                   width_m: float = 1.0,
                   step_limit_m: float = 0.1068,
                   # Local neighborhood & Z slice
                   local_radius_m: float = 1.0,
                   z_center_m: float = 0.0,
                   z_half_thickness_m: float = 1.0,
                   forward_only: int = 1,
                   # Window toggles
                   window: int = 1,     # XY window
                   z_window: int = 1,   # Z slice
                   # Slope method
                   method: str = "column",  # "column" | "plane"
                   # Radial cliff detection options
                   radial_bins: int = 16,
                   cliff_sample_m: float = 1.0,
                   cliff_first_n: int = 3,
                   cliff_limit_deg: float = 45.0,
                   return_sectors: int = 1,
                   debug: int = 0):
    """
    Traversability + Cliff detection.

    OK (traverse): abs(pitch_deg_used) <= climb_limit_deg AND max_step_m <= step_limit_m
    OK_cliff:      NO direction (within sampling range) whose initial slope >= cliff_limit_deg

    Windowing:
      window=1: XY neighborhood (forward-only: X∈[0..ahead_m], |Y|<=width/2; else bubble |X|,|Y|<=local_radius_m)
      window=0: full XY
      z_window=1: Z∈[z_center ± z_half]; z_window=0: full Z

    Slope method:
      - "column": pitch from first/last valid X columns (fast, quantized)
      - "plane":  least-squares plane fit -> stable slope magnitude

    Cliff detection (radial):
      - For each azimuth bin, sample first_n valid voxels within 'cliff_sample_m' from the sensor and
        compute initial slope angle; classify vs 'cliff_limit_deg'.
    """
    a = _get_voxel_adapter(adapter_id)
    g = a.grid
    voxel = a.voxel

    # XY window
    if int(window) == 1:
        ix0 = int((0.0 - a.xmin) / voxel)
        iy0 = int((0.0 - a.ymin) / voxel)
        if int(forward_only) == 1:
            dx = max(1, int(round(float(ahead_m) / voxel)))
            wy = max(1, int(round(float(width_m) / voxel)))
            ix_start = max(0, ix0); ix_end = min(a.nx - 1, ix0 + dx)
            iy_start = max(0, iy0 - wy // 2); iy_end = min(a.ny - 1, iy0 + wy // 2)
        else:
            rpx = max(1, int(round(local_radius_m / voxel)))
            ix_start = max(0, ix0 - rpx); ix_end = min(a.nx - 1, ix0 + rpx)
            iy_start = max(0, iy0 - rpx); iy_end = min(a.ny - 1, iy0 + rpx)
    else:
        ix_start, ix_end = 0, a.nx - 1
        iy_start, iy_end = 0, a.ny - 1

    # Z window
    if int(z_window) == 1:
        zc = float(z_center_m)
        zh = max(0.01, float(z_half_thickness_m))
        iz0 = int(max(0, math.floor(((zc - zh) - a.zmin) / voxel)))
        iz1 = int(min(a.nz - 1, math.ceil(((zc + zh) - a.zmin) / voxel)))
    else:
        iz0, iz1 = 0, a.nz - 1

    # Collect heights (max z per column)
    xs: List[float] = []
    hs: List[float] = []
    for ix in range(ix_start, ix_end + 1):
        h_col = []
        for iy in range(iy_start, iy_end + 1):
            hz = _column_max_z(g, ix, iy, a.zmin, voxel, iz0=iz0, iz1=iz1)
            if hz is not None:
                h_col.append(hz)
        if h_col:
            xs.append(a.xmin + ix * voxel)
            hs.append(max(h_col))

    if len(hs) < 3:
        out = {
            "pitch_deg": None,
            "max_step_m": None,
            "ok": False,
            "ok_cliff": None,
            "note": "no occupancy in selected window",
            "window": bool(int(window)),
            "z_window": bool(int(z_window)),
            "forward_only": bool(int(forward_only)),
            "method": method
        }
        return Response(content=json.dumps(out), media_type="application/json")

    # Column method: pitch & step
    x_span = (xs[-1] - xs[0]) if len(xs) > 1 else voxel
    dz = (hs[-1] - hs[0])
    pitch_deg_col = math.degrees(math.atan2(dz, max(x_span, 1e-3)))
    max_step = 0.0
    for i in range(1, len(hs)):
        dstep = abs(hs[i] - hs[i - 1])
        if dstep > max_step:
            max_step = dstep

    # Plane-fit (optional)
    pitch_deg_used = pitch_deg_col
    if method.lower() == "plane":
        x0w = a.xmin + ix_start * voxel; x1w = a.xmin + ix_end * voxel
        y0w = a.ymin + iy_start * voxel; y1w = a.ymin + iy_end * voxel
        z0w = a.zmin + iz0 * voxel;      z1w = a.zmin + iz1 * voxel
        H, xcoords, ycoords = a._heightmap_from_window(x0w, x1w, y0w, y1w, z0w, z1w)
        mask = np.isfinite(H)
        if mask.any():
            xi, yi = np.where(mask)
            xw = xcoords[xi]; yw = ycoords[yi]; zw = H[mask]
            A = np.stack([xw, yw, np.ones_like(xw)], axis=1)
            try:
                coeff, *_ = np.linalg.lstsq(A, zw, rcond=None)
                ax, by, c = [float(coeff[0]), float(coeff[1]), float(coeff[2])]
                slope = math.sqrt(ax*ax + by*by)  # tan(theta)
                pitch_plane = math.degrees(math.atan(slope))
                # keep sign consistent with column direction
                pitch_deg_used = pitch_plane if pitch_deg_col >= 0 else -pitch_plane
            except Exception:
                pass

    # Traverse OK decision (your intended logic)
    ok = (abs(pitch_deg_used) <= a.climb_limit_deg) and (max_step <= step_limit_m)

    # --- Cliff detection via radials ---
    sectors = a._radial_slopes(
        radial_bins=int(radial_bins),
        sample_m=float(cliff_sample_m),
        first_n=int(cliff_first_n),
        x0=0.0, y0=0.0,
        iz0=iz0, iz1=iz1,
        forward_only=bool(int(forward_only))
    )
    # OK_cliff means no sector exceeds cliff_limit_deg
    cliff_hits = [s for s in sectors if math.isfinite(s["pitch_deg"]) and abs(s["pitch_deg"]) >= float(cliff_limit_deg)]
    ok_cliff = len(cliff_hits) == 0

    out = {
        "pitch_deg": round(pitch_deg_used, 2),
        "max_step_m": round(max_step, 3),
        "ok": bool(ok),                     # traversability OK
        "ok_cliff": bool(ok_cliff),         # no cliff detected in sampled directions
        "climb_limit_deg": a.climb_limit_deg,
        "step_limit_m": step_limit_m,
        "samples": len(hs),
        "forward_only": bool(int(forward_only)),
        "window": bool(int(window)),
        "z_window": bool(int(z_window)),
        "method": method,
        "x_window_m": [round(a.xmin + ix_start * voxel, 3), round(a.xmin + ix_end * voxel, 3)],
        "y_window_m": [round(a.ymin + iy_start * voxel, 3), round(a.ymin + iy_end * voxel, 3)],
        "z_slice_m": [round(a.zmin + iz0 * voxel, 3), round(a.zmin + iz1 * voxel, 3)],
        "cliff": {
            "radial_bins": int(radial_bins),
            "sample_m": float(cliff_sample_m),
            "first_n": int(cliff_first_n),
            "limit_deg": float(cliff_limit_deg),
            "hits": len(cliff_hits)
        }
    }


# src/sensorhub/adapters/livox_mid360/livox_voxel_adapter.py
"""
Livox -> IMU-corrected -> 3D voxel grid accumulator (8-bit per voxel)

- 0.025 m voxel size (configurable)
- ±20 m XY, Z=[-2, +8] m (configurable)
- Occupancy strength projection (top-down)
- BINVOX v2 export (stores 0..255 voxel values, not only binary)
- WebSocket stream for top-down tiles
- Traversability probe endpoint

NEW:
- /livox_voxel/topdown_filtered.png -> local slice (default: radius 1 m, z ∈ [-1, +1] around sensor)
- /livox_voxel/traverse/check -> refined to use local 1 m neighborhood (configurable) and corrected `ok` logic.
"""
import math, time, threading, logging, json, struct, asyncio
from typing import Dict, Tuple, Optional, List
import numpy as np
from fastapi import APIRouter, HTTPException, Response, WebSocket, WebSocketDisconnect
from sensorhub.core.sensor_base import AbstractSensorAdapter
from sensorhub.core.sensor_manager import manager

try:
    from PIL import Image
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
                 voxel_size_m: float = 0.025,
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
        self._pts: List[Tuple[float, float, float, int, float]] = []  # (x,y,z,i, ts)

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

        # Transform hooks (YAML)
        # self.set_transform(...) is inherited; we apply in _apply_transform() if present.

    # --- abstract lifecycle ---
    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        self._stop.set()
        super().stop()

    # --- helpers ---
    def _apply_transform(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Apply YAML Transform if present (rotation, scale, translation).
        Transform may be a dataclass or raw dict.
        """
        t = getattr(self, "transform", None)
        if t is None:
            return x, y, z
        try:
            # Minimal: scale + RPY + translation (degrees -> radians)
            sx = getattr(t, "scale", 1.0) if hasattr(t, "scale") else float(t.get("scale", 1.0))
            rx = math.radians(getattr(t, "roll_deg", 0.0) if hasattr(t, "roll_deg") else float(t.get("roll_deg", 0.0)))
            ry = math.radians(getattr(t, "pitch_deg", 0.0) if hasattr(t, "pitch_deg") else float(t.get("pitch_deg", 0.0)))
            rz = math.radians(getattr(t, "yaw_deg", 0.0) if hasattr(t, "yaw_deg") else float(t.get("yaw_deg", 0.0)))
            tx = getattr(t, "tx", 0.0) if hasattr(t, "tx") else float(t.get("tx", 0.0))
            ty = getattr(t, "ty", 0.0) if hasattr(t, "ty") else float(t.get("ty", 0.0))
            tz = getattr(t, "tz", 0.0) if hasattr(t, "tz") else float(t.get("tz", 0.0))

            # Scale
            x, y, z = sx * x, sx * y, sx * z

            # Rotation (Z * Y * X)
            cz, sz = math.cos(rz), math.sin(rz)
            cy, sy = math.cos(ry), math.sin(ry)
            cx, sx_ = math.cos(rx), math.sin(rx)

            # Apply rotations
            # Rz
            x, y = (cz * x - sz * y), (sz * x + cz * y)
            # Ry
            x, z = (cy * x + sy * z), (-sy * x + cy * z)
            # Rx
            y, z = (cx * y - sx_ * z), (sx_ * y + cx * z)

            # Translation
            x, y, z = x + tx, y + ty, z + tz
            return x, y, z
        except Exception:
            return x, y, z

    def _imu_rotation_quat(self) -> Tuple[float, float, float, float]:
        """Small-angle quaternion from gyro if above threshold."""
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
        """Rotate (x,y,z) by quaternion q (small-angle approx OK)."""
        w, qx, qy, qz = q
        vx, vy, vz = x, y, z
        # q_vec x v
        cx = qy*vz - qz*vy
        cy = qz*vx - qx*vz
        cz = qx*vy - qy*vx
        vx += 2.0 * cx
        vy += 2.0 * cy
        vz += 2.0 * cz
        return vx, vy, vz

    def _insert_voxel(self, xr: float, yr: float, zr: float, intensity: int) -> None:
        if not (self.xmin <= xr <= self.xmax and self.ymin <= yr <= self.ymax and self.zmin <= zr <= self.zmax):
            return
        ix = int((xr - self.xmin) / self.voxel)
        iy = int((yr - self.ymin) / self.voxel)
        iz = int((zr - self.zmin) / self.voxel)
        v = self.grid[ix, iy, iz]
        hits = (v & 0x1F) + 1
        v = (0x80) | min(31, hits)   # occupied + saturating strength
        # Optional class flags (surface vs obstacle). Simple heuristic:
        if zr > 0.3:
            v = (v & 0x9F) | (0b10 << 5)  # obstacle
        else:
            v = (v & 0x9F) | (0b01 << 5)  # surface/ground
        self.grid[ix, iy, iz] = v
        self._topdown_dirty = True

    def _update_topdown(self) -> None:
        """Project max strength across z if any occupied in column (full volume)."""
        g = self.grid
        occ = (g & 0x80) > 0
        strength = (g & 0x1F)
        max_s = np.where(occ.any(axis=2), strength.max(axis=2), 0)
        self._topdown[:] = max_s.astype(np.uint8)
        self._topdown_dirty = False

    # --- NEW: local slice topdown projector ---
    def _topdown_from_window(self,
                             x_min: float, x_max: float,
                             y_min: float, y_max: float,
                             z_min: float, z_max: float) -> np.ndarray:
        """
        Build a 2D top-down (uint8) strength map from a bounded 3D window.
        Returns an array covering the [x_min..x_max] x [y_min..y_max] indices.
        """
        # Clamp world-space window into grid
        x_min = max(self.xmin, x_min); x_max = min(self.xmax, x_max)
        y_min = max(self.ymin, y_min); y_max = min(self.ymax, y_max)
        z_min = max(self.zmin, z_min); z_max = min(self.zmax, z_max)
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
            return np.zeros((1, 1), dtype=np.uint8)

        ix0 = int((x_min - self.xmin) / self.voxel)
        ix1 = int((x_max - self.xmin) / self.voxel)
        iy0 = int((y_min - self.ymin) / self.voxel)
        iy1 = int((y_max - self.ymin) / self.voxel)
        iz0 = int((z_min - self.zmin) / self.voxel)
        iz1 = int((z_max - self.zmin) / self.voxel)

        ix1 = min(ix1, self.nx - 1)
        iy1 = min(iy1, self.ny - 1)
        iz1 = min(iz1, self.nz - 1)

        # slice and project
        g = self.grid[ix0:ix1+1, iy0:iy1+1, iz0:iz1+1]
        if g.size == 0:
            return np.zeros((1, 1), dtype=np.uint8)
        occ = (g & 0x80) > 0
        strength = (g & 0x1F)
        proj = np.where(occ.any(axis=2), strength.max(axis=2), 0).astype(np.uint8)
        return proj

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
                    # motion + transform
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

                # publish status periodically
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
    # Advertise routes for Summary page
    return {
        "adapter_id": adapter_id,
        "routes": {
            "meta": "/livox_voxel/meta",
            "binvox": "/livox_voxel/grid.binvox",
            "topdown_raw": "/livox_voxel/topdown.raw",
            "topdown_png": "/livox_voxel/topdown.png",
            "topdown_filtered": "/livox_voxel/topdown_filtered.png",  # NEW
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
    """
    BINVOX v2: value/count pairs, value in [0..255]; not just binary occupancy.
    """
    a = _get_voxel_adapter(adapter_id)
    g = a.grid  # uint8
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
    """
    Raw uint8 [nx * ny] row-major occupancy strength (top-down), with headers.
    """
    a = _get_voxel_adapter(adapter_id)
    raw = a.topdown_bytes()
    headers = {
        "X-Width": str(a.nx),
        "X-Height": str(a.ny),
        "X-Voxel-M": str(a.voxel)
    }
    return Response(content=raw, media_type="application/octet-stream", headers=headers)


@router.get("/topdown.png", response_class=Response)
def topdown_png(
    adapter_id: str = "livox_voxel",
    # --- visibility controls ---
    scale_mode: str = "auto",  # "linear" | "auto" | "equalize"
    gain: float = 8.0,
    clip_min: int = 0,
    clip_max: int = 31,
    cmap: str = "gray",  # "gray" | "hot" | "viridis"
    # --- overlays ---
    draw_grid: int = 0,  # <- default OFF
    tick_m: float = 1.0,
    mark_center: int = 1,
    invert_y: int = 0,
    # --- framing / zoom ---
    crop: int = 0,       # occupied bbox crop (0/1)
    crop_radius_m: float = 10.0,  # center crop radius (meters). <=0 to disable.
    downscale: int = 2   # 1=no scale, 2=half, 4=quarter (bilinear)
):
    """
    PNG (colorized) of top-down occupancy strength with scaling, optional center crop,
    optional occupied-bbox crop, and optional downscale for UI.
    """
    a = _get_voxel_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")

    # Get latest top-down
    if a._topdown_dirty:
        a._update_topdown()
    td = a._topdown  # uint8 0..31

    # Optional flip
    arr = td if not invert_y else np.flipud(td)
    arr = arr.astype(np.float32)
    arr = np.clip(arr, float(clip_min), float(clip_max))

    # Contrast mapping
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
    else:  # linear
        arr = np.clip(arr * gain * (255.0 / 31.0), 0.0, 255.0)

    if scale_mode != "equalize":
        img = Image.fromarray(arr.astype(np.uint8), mode="L")

    # Colormap → RGB for drawing
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

    # Dimensions before cropping (full raster orientation after invert)
    h_full, w_full = arr.shape[0], arr.shape[1]

    # --- Center crop by radius (meters) ---
    crop_offset_x = 0
    crop_offset_y = 0
    if crop_radius_m and crop_radius_m > 0:
        rpx = int(round(crop_radius_m / a.voxel))
        # compute full center in pixels
        cx_full = int(round((0.0 - a.xmin) / a.voxel))
        cy_full = int(round((0.0 - a.ymin) / a.voxel))
        if invert_y:
            cy_full = h_full - 1 - cy_full
        # bounds
        x0 = max(0, cx_full - rpx); x1 = min(w_full - 1, cx_full + rpx)
        y0 = max(0, cy_full - rpx); y1 = min(h_full - 1, cy_full + rpx)
        img = img.crop((x0, y0, x1 + 1, y1 + 1))
        crop_offset_x, crop_offset_y = x0, y0
        w_full, h_full = img.size[0] + x0, img.size[1] + y0  # reset reference if needed

    # --- Occupied bbox crop (if asked) ---
    if crop and (crop_radius_m <= 0):
        if a._topdown_dirty:
            a._update_topdown()
        td_full = a._topdown
        occ = (td_full > 0)
        occ = np.flipud(occ) if invert_y else occ
        if occ.any():
            ys, xs = np.where(occ)
            y0, y1 = int(ys.min()), int(ys.max())
            x0, x1 = int(xs.min()), int(xs.max())
            y0 = max(0, y0 - 4); x0 = max(0, x0 - 4)
            y1 = min(img.size[1] - 1, y1 + 4)
            x1 = min(img.size[0] - 1, x1 + 4)
            img = img.crop((x0, y0, x1 + 1, y1 + 1))
            crop_offset_x += x0; crop_offset_y += y0

    # Draw overlays on RGB
    from PIL import ImageDraw
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
        cx = max(0, min(w - 1, cx_full - crop_offset_x))
        cy = max(0, min(h - 1, cy_full - crop_offset_y))
        draw.line([(cx - 6, cy), (cx + 6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy - 6), (cx, cy + 6)], fill=(255, 0, 0), width=2)

    # --- Downscale for UI (bilinear) ---
    if downscale and int(downscale) > 1:
        factor = int(downscale)
        nw = max(1, w // factor); nh = max(1, h // factor)
        img = img.resize((nw, nh), resample=Image.BILINEAR)
        w, h = nw, nh

    # Encode
    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {
        "X-Width": str(w),
        "X-Height": str(h),
        "X-Voxel-M": str(a.voxel)
    }
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)


# --- NEW: filtered local topdown (radius ±1 m; Z slice ±1 m) ---
@router.get("/topdown_filtered.png", response_class=Response)
def topdown_filtered_png(
    adapter_id: str = "livox_voxel",
    radius_m: float = 1.0,
    z_center_m: float = 0.0,
    z_half_thickness_m: float = 1.0,
    # visualization options consistent with /topdown.png
    scale_mode: str = "auto",
    gain: float = 8.0,
    clip_min: int = 0,
    clip_max: int = 31,
    cmap: str = "gray",
    draw_grid: int = 1,
    tick_m: float = 0.25,
    mark_center: int = 1,
    invert_y: int = 0,
    downscale: int = 1
):
    """
    PNG of a local top-down slice centered at the sensor:
    - XY within `radius_m` (default 1.0 m).
    - Z within [z_center_m - z_half_thickness_m, z_center_m + z_half_thickness_m] (default [-1, +1] m).
    This filters out ceiling/far structures while showing immediate surroundings.
    """
    a = _get_voxel_adapter(adapter_id)
    if not PIL_OK:
        raise HTTPException(500, "Pillow not available for PNG export")

    r = max(0.01, float(radius_m))
    zc = float(z_center_m)
    zh = max(0.01, float(z_half_thickness_m))

    # Build local window in world coordinates
    x0, x1 = -r, +r
    y0, y1 = -r, +r
    z0, z1 = zc - zh, zc + zh

    proj = a._topdown_from_window(x0, x1, y0, y1, z0, z1)
    if invert_y:
        proj = np.flipud(proj)

    arr = proj.astype(np.float32)
    arr = np.clip(arr, float(clip_min), float(clip_max))

    # Contrast / color
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

    # Colormap
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

    # Overlays
    from PIL import ImageDraw
    draw = ImageDraw.Draw(img)
    w, h = img.size
    if draw_grid:
        spacing_px = max(1, int(round(tick_m / a.voxel)))
        col = (200, 200, 200) if cmap == "gray" else (255, 255, 255)
        for x in range(0, w, spacing_px):
            draw.line([(x, 0), (x, h - 1)], fill=col, width=1)
        for y in range(0, h, spacing_px):
            draw.line([(0, y), (w - 1, y)], fill=col, width=1)
    if mark_center:
        cx = max(0, min(w - 1, w // 2))
        cy = max(0, min(h - 1, h // 2))
        draw.line([(cx - 6, cy), (cx + 6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy - 6), (cx, cy + 6)], fill=(255, 0, 0), width=2)

    # Downscale
    if downscale and int(downscale) > 1:
        factor = int(downscale)
        nw = max(1, w // factor); nh = max(1, h // factor)
        img = img.resize((nw, nh), resample=Image.BILINEAR)
        w, h = nw, nh

    # Encode
    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {
        "X-Width": str(w),
        "X-Height": str(h),
        "X-Voxel-M": str(a.voxel),
        "X-Window": json.dumps({"x": [-r, +r], "y": [-r, +r], "z": [z0, z1]}),
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
    """
    Returns the maximum occupied Z (world meters) in a column slice [iz0..iz1].
    If iz0/iz1 are None, the whole column is considered.
    """
    col = grid[ix, iy, :]
    if iz0 is not None or iz1 is not None:
        # clamp slice
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
                   ahead_m: float = 2.0,
                   width_m: float = 0.5,
                   step_limit_m: float = 0.1068,
                   # NEW: focus on immediate area around the sensor
                   local_radius_m: float = 1.0,
                   z_center_m: float = 0.0,
                   z_half_thickness_m: float = 1.0):
    """
    Check traversability using the local neighborhood around the sensor.

    Changes vs. original:
    - Uses only the XY neighborhood within `local_radius_m` (default 1.0 m) around the sensor,
      instead of far-lookahead being the primary source.
    - Restricts Z to [z_center_m - z_half_thickness_m, z_center_m + z_half_thickness_m]
      (default [-1, +1] m) to ignore ceiling/overhead clutter.
    - `ok` condition corrected to: abs(pitch_deg) <= climb_limit_deg AND max_step_m <= step_limit_m.

    For compatibility, `ahead_m`/`width_m` are still accepted but the local radius is dominant.
    """
    a = _get_voxel_adapter(adapter_id)
    g = a.grid  # (nx, ny, nz)
    voxel = a.voxel

    ix0 = int((0.0 - a.xmin) / voxel)  # center X
    iy0 = int((0.0 - a.ymin) / voxel)  # center Y

    rpx = max(1, int(round(local_radius_m / voxel)))
    # Define local square window around center (approx of cylinder)
    ix_start = max(0, ix0 - rpx); ix_end = min(a.nx - 1, ix0 + rpx)
    iy_start = max(0, iy0 - rpx); iy_end = min(a.ny - 1, iy0 + rpx)

    # Z slice for local interaction (filters ceiling)
    zc = float(z_center_m)
    zh = max(0.01, float(z_half_thickness_m))
    iz0 = int(max(0, math.floor(((zc - zh) - a.zmin) / voxel)))
    iz1 = int(min(a.nz - 1, math.ceil(((zc + zh) - a.zmin) / voxel)))

    xs: List[float] = []
    hs: List[float] = []  # height per x (max across lateral band in local window)

    for ix in range(ix_start, ix_end + 1):
        h_col = []
        for iy in range(iy_start, iy_end + 1):
            hz = _column_max_z(g, ix, iy, a.zmin, voxel, iz0=iz0, iz1=iz1)
            if hz is not None:
                h_col.append(hz)
        if h_col:
            xs.append(a.xmin + ix * voxel)
            hs.append(max(h_col))

    has_data = len(hs) >= 3
    if not has_data:
        out = {
            "pitch_deg": None,
            "max_step_m": None,
            "ok": False,
            "note": "no local occupancy within 1 m neighborhood"
        }
        return Response(content=json.dumps(out), media_type="application/json")

    # pitch from min/max heights along x
    x_span = (xs[-1] - xs[0]) if len(xs) > 1 else voxel
    dz = (hs[-1] - hs[0])
    pitch_deg = math.degrees(math.atan2(dz, max(x_span, 1e-3)))

    # max step between consecutive x samples
    max_step = 0.0
    for i in range(1, len(hs)):
        dstep = abs(hs[i] - hs[i - 1])
        if dstep > max_step:
            max_step = dstep

    # CORRECTED: use <= climb limit (instead of >=)
    ok = (abs(pitch_deg) <= a.climb_limit_deg) and (max_step <= step_limit_m)

    out = {
        "pitch_deg": round(pitch_deg, 2),
        "max_step_m": round(max_step, 3),
        "ok": bool(ok),
        "climb_limit_deg": a.climb_limit_deg,
        "step_limit_m": step_limit_m,
        "samples": len(hs),
        "local_radius_m": local_radius_m,
        "z_slice_m": [round(zc - zh, 3), round(zc + zh, 3)]
    }
    return Response(content=json.dumps(out), media_type="application/json")

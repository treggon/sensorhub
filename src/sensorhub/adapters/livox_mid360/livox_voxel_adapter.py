
# src/sensorhub/adapters/livox_mid360/livox_voxel_adapter.py
"""
Livox -> IMU-corrected -> 3D voxel grid accumulator (8-bit per voxel)

Voxel code (uint8):
  bit7     : occupied (1=occupied)
  bits6..5 : class (00=unknown, 01=surface/ground, 10=obstacle, 11=reserved)
  bits4..0 : saturating strength (0..31)

Key behaviors:
- Transform is applied to Livox points **before** voxelization.
- Changing the transform **clears** voxel memory immediately (new inserts reflect new pose).
- Decay system (auto + manual) to fade old voxels out.
- Top-down PNG with optional display rotation (heading_deg).
- Filtered local top-down with directional colorized rays, inner gap, and grid toggle.
- Traversability probe (column/plane) + directional cliff detection.
- UI-friendly traversal summary endpoint returning decisions + image URL.
- WebSocket stream for raw top-down tiles.

Health model (adapter-level):
- ok      : fresh voxel updates in < ok_stale_sec
- warning : running but no recent updates (< err_stale_sec) OR upstream warning
- error   : adapter not running OR upstream error/unavailable OR updates stale ≥ err_stale_sec
"""
import math
import time
import threading
import logging
import json
import struct
import asyncio
from typing import Dict, Tuple, Optional, List

import numpy as np
from fastapi import APIRouter, HTTPException, Response, WebSocket, WebSocketDisconnect
from sensorhub.core.sensor_base import AbstractSensorAdapter
from sensorhub.core.sensor_manager import manager

try:
    from PIL import Image
    from PIL import ImageDraw, ImageOps
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
    """
    def __init__(self,
                 sensor_id: str,
                 kind: str = "voxelgrid",
                 source_id: str = "livox",
                 voxel_size_m: float = 0.0125,
                 grid_xy_m: float = 20.0,
                 grid_z_m: Tuple[float, float] = (-2.0, 8.0),
                 chunk_size: int = 32,
                 imu_threshold_radps: float = 0.01,
                 min_hits_for_occupied: int = 3,
                 publish_hz: Optional[float] = 2.0,
                 climb_limit_deg: float = 45.0,
                 # --- NEW health knobs ---
                 ok_stale_sec: float = 1.0,
                 warn_stale_sec: float = 3.0,
                 err_stale_sec: float = 10.0,
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
        self.nx = int(math.ceil((+self.xy - (-self.xy)) / self.voxel))
        self.ny = int(math.ceil((+self.xy - (-self.xy)) / self.voxel))
        self.nz = int(math.ceil((self.zmax - self.zmin) / self.voxel))
        self.xmin, self.xmax = -self.xy, +self.xy
        self.ymin, self.ymax = -self.xy, +self.xy

        # Data
        self.grid = np.zeros((self.nx, self.ny, self.nz), dtype=np.uint8)

        # Transform (applied before voxelization)
        self.transform: Dict[str, float] = {
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
            "tx": 0.0, "ty": 0.0, "tz": 0.0, "scale": 1.0
        }

        # IMU / motion compensation (disabled by default; small-angle quat if enabled)
        self.imu_eps = float(imu_threshold_radps)
        self._imu: Optional[Dict[str, float]] = None
        self.use_gyro_rotation = False  # keep OFF unless accurate timestamps are provided
        self.last_imu_ts: Optional[float] = None

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

        # Decay controls (defaults: on, every 5s, alpha 0.95)
        self.enable_decay: bool = bool(kwargs.get("enable_decay", True))
        self.decay_alpha: float = float(kwargs.get("decay_alpha", 0.95))
        self.decay_period_s: float = float(kwargs.get("decay_period_s", 5.0))
        self._last_decay: float = time.time()

        # --- NEW health tracking ---
        self.ok_stale_sec = float(ok_stale_sec)
        self.warn_stale_sec = float(warn_stale_sec)
        self.err_stale_sec = float(err_stale_sec)
        self._last_update_ts: float = 0.0       # last time grid was updated from incoming points
        self._last_source_ts: float = 0.0       # timestamp provided by upstream (if any)
        self._last_run_error: Optional[str] = None

    # --- lifecycle ---
    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        self._stop.set()
        super().stop()

    # --- transform setter (clears grid when changed) ---
    def set_transform(self, t) -> None:
        """
        Accepts either a dict-like or an object with attributes roll_deg/pitch_deg/yaw_deg/tx/ty/tz/scale.
        Update transform and CLEAR voxel memory so new inserts reflect the new pose.
        """
        def _get(src, key, default):
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
        self.grid[:] = 0
        self._topdown_dirty = True
        self._last_update_ts = 0.0
        self.log.info("Transform updated; voxel grid cleared.")

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
        """
        Return identity unless gyro rotation is enabled with a valid dt.
        """
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

    # --- insert & projection ---
    def _insert_voxel(self, xr: float, yr: float, zr: float, intensity: int) -> None:
        """
        Half-open upper bounds and index clamping to prevent OOB when a point lies on xmax/ymax/zmax.
        """
        # Use half-open interval on the upper bounds to avoid ix==nx, etc.
        if not (self.xmin <= xr < self.xmax and
                self.ymin <= yr < self.ymax and
                self.zmin <= zr < self.zmax):
            return

        # Compute indices
        ix = int((xr - self.xmin) / self.voxel)
        iy = int((yr - self.ymin) / self.voxel)
        iz = int((zr - self.zmin) / self.voxel)

        # Clamp for absolute safety (handles any roundoff/edge cases)
        if ix < 0: ix = 0
        elif ix >= self.nx: ix = self.nx - 1
        if iy < 0: iy = 0
        elif iy >= self.ny: iy = self.ny - 1
        if iz < 0: iz = 0
        elif iz >= self.nz: iz = self.nz - 1

        v = self.grid[ix, iy, iz]
        hits = (v & 0x1F) + 1
        v = (0x80) | min(31, hits)   # occupied + saturating strength

        # simple classing by height above 0; adjust if your ground reference differs
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

    # --- decay ---
    def decay(self, alpha: Optional[float] = None, min_strength: int = 0) -> Dict[str, int]:
        """
        Exponential decay of voxel 'strength' (bits 4..0). Occupancy (bit7) is cleared
        when the decayed strength <= min_strength. Class bits (bits6..5) are preserved.
        alpha in (0,1]: e.g., 0.95 -> ~5% fade per decay step.
        Returns counters: {'kept':N, 'cleared':M}.
        """
        a = float(self.decay_alpha if alpha is None else alpha)
        a = max(0.0, min(1.0, a))
        thr = int(max(0, min(31, min_strength)))

        v = self.grid  # uint8 array
        occ = (v & 0x80) > 0
        cls = (v & 0x60)           # preserve class bits
        s0  = (v & 0x1F).astype(np.uint8)

        s1 = (s0.astype(np.float32) * a).astype(np.uint8)

        clear_mask = occ & (s1 <= thr)
        keep_mask  = occ & (s1 >  thr)

        v_new = np.zeros_like(v, dtype=np.uint8)
        v_new[keep_mask] = (0x80 | cls[keep_mask] | (s1[keep_mask] & 0x1F)).astype(np.uint8)
        self.grid[:] = v_new
        self._topdown_dirty = True

        kept = int(np.count_nonzero(keep_mask))
        cleared = int(np.count_nonzero(clear_mask))
        return {"kept": kept, "cleared": cleared}

    # --- main loop (resilient) ---
    def run(self) -> None:
        self.log.info("LivoxVoxelAdapter run() loop.")
        try:
            while not self._stop.is_set():
                try:
                    # ---- begin protected block ----
                    sample = manager.latest(self.source_id)
                    if sample and isinstance(sample.data, dict):
                        src_ts = sample.data.get("timestamp")
                        if isinstance(src_ts, (int, float)):
                            self._last_source_ts = float(src_ts)

                        pts = sample.data.get("points") or []
                        imu = sample.data.get("imu")
                        if imu:
                            self._imu = imu

                        # For robot-centric accumulation, leave q as identity unless you’re certain about dt
                        q = (1.0, 0.0, 0.0, 0.0) if not self.use_gyro_rotation else self._imu_rotation_quat()

                        any_update = False
                        for p in pts:
                            if len(p) < 2:
                                continue
                            if len(p) == 2:
                                x, y = float(p[0]), float(p[1]); z, i = 0.0, 0
                            elif len(p) == 3:
                                x, y, z = float(p[0]), float(p[1]), float(p[2]); i = 0
                            else:
                                x, y, z, i = float(p[0]), float(p[1]), float(p[2]), int(p[3])

                            # Apply IMU quaternion (usually identity), then the sensor transform
                            x, y, z = self._apply_quat(x, y, z, q)
                            xr, yr, zr = self._apply_transform(x, y, z)

                            # Insert transformed point into voxel grid (now boundary-safe)
                            self._insert_voxel(xr, yr, zr, i)
                            any_update = True

                        if any_update:
                            self._last_update_ts = time.time()

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

                    # Auto-decay every self.decay_period_s
                    try:
                        if self.enable_decay and (time.time() - self._last_decay) >= float(self.decay_period_s):
                            _ = self.decay(self.decay_alpha)
                            self._last_decay = time.time()
                    except Exception:
                        # keep running, but record last error
                        self._last_run_error = "decay step failed"
                        self.log.exception("Voxel decay step failed")
                    # ---- end protected block ----

                except Exception as e:
                    # Capture and keep running
                    self._last_run_error = f"{e.__class__.__name__}: {e}"
                    self.log.exception("Unhandled exception in voxel run-loop; continuing")
                    time.sleep(0.05)
        finally:
            self.log.info("LivoxVoxelAdapter exit.")

    # --- public snapshots ---
    def topdown_bytes(self) -> bytes:
        if self._topdown_dirty:
            self._update_topdown()
        return self._topdown.tobytes(order="C")

    # =========================
    # NEW: health & readiness
    # =========================
    def _last_update_age_sec(self) -> Optional[float]:
        """Age (seconds) since the grid was last updated from upstream points. None => never updated."""
        if self._last_update_ts <= 0.0:
            return None
        return max(0.0, time.time() - self._last_update_ts)

    def _valid_ratio(self) -> float:
        """Fraction of XY cells that have any occupied voxel."""
        g = self.grid
        occ_xy = (g & 0x80).any(axis=2)
        total = occ_xy.size
        if total <= 0:
            return 0.0
        return float(np.count_nonzero(occ_xy)) / float(total)

    def is_ready(self) -> bool:
        """Ready when the grid has recent updates (freshness threshold == warn_stale_sec)."""
        age = self._last_update_age_sec()
        return (age is not None) and (age < self.warn_stale_sec)

    def health(self) -> dict:
        """
        Report adapter health with ok/warning/error classification.
        Takes upstream (source_id) status into account but does not hard-fail
        unless upstream is explicitly in 'error' OR local grid is long-stale.
        """
        now = time.time()
        running = self.is_running()
        age = self._last_update_age_sec()  # None if never updated
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

        if not running:
            status = "error"
            reason = f"adapter not running: {self._last_run_error}" if self._last_run_error else "adapter not running"
        else:
            if upstream_status == "error":
                if age is None or age >= self.ok_stale_sec:
                    status = "error"
                    reason = "upstream error"
                else:
                    status = "warning"
                    reason = "upstream error; using cached grid"
            else:
                if age is None:
                    if upstream_status in ("ok", "warning", None):
                        status = "warning"
                        reason = "no voxel updates yet; awaiting points"
                    else:
                        status = "error"
                        reason = f"upstream {upstream_status}"
                elif age < self.ok_stale_sec:
                    status = "ok"
                    reason = "fresh grid"
                elif age < self.err_stale_sec:
                    status = "warning"
                    reason = f"grid stale ({age:.1f}s)"
                else:
                    status = "error"
                    reason = f"no voxel updates for {age:.1f}s"

        return {
            "id": self.sensor_id,
            "kind": self.kind,
            "status": status,                      # ui: green / yellow / red
            "reason": reason,
            "running": running,
            "ready": (status == "ok"),
            "last_update_ts": self._last_update_ts if self._last_update_ts > 0 else None,
            "last_data_age_sec": None if age is None else round(age, 2),
            "valid_ratio": round(vr, 4),
            "upstream": upstream if isinstance(upstream, dict) else None,
            "timestamp": now,
        }


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
            "traverse_check": "/livox_voxel/traverse/check",
            "traverse_summary": "/livox_voxel/traverse/summary",
            "transform_get": "/livox_voxel/transform",
            "transform_set": "/livox_voxel/transform",
            "transform_apply_and_clear": "/livox_voxel/transform/apply_and_clear",
            "clear": "/livox_voxel/clear",
            "decay_config_get": "/livox_voxel/decay/config",
            "decay_config_set": "/livox_voxel/decay/config",
            "decay_once": "/livox_voxel/decay"
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
        "climb_limit_deg": a.climb_limit_deg,
        "decay": {
            "enable_decay": a.enable_decay,
            "decay_alpha": a.decay_alpha,
            "decay_period_s": a.decay_period_s
        },
        "transform": a.transform
    }

# --- Transform endpoints (setting transform clears grid) ---
@router.get("/transform")
def get_transform(adapter_id: str = "livox_voxel"):
    a = _get_voxel_adapter(adapter_id)
    return {"ok": True, "sensor_id": adapter_id, "transform": a.transform}

@router.post("/transform")
def set_transform(adapter_id: str = "livox_voxel",
                  roll_deg: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0,
                  tx: float = 0.0, ty: float = 0.0, tz: float = 0.0,
                  scale: float = 1.0):
    a = _get_voxel_adapter(adapter_id)
    a.set_transform({
        "roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg,
        "tx": tx, "ty": ty, "tz": tz, "scale": scale
    })
    return {"ok": True, "sensor_id": adapter_id, "transform": a.transform}

@router.post("/transform/apply_and_clear")
def set_transform_and_clear(adapter_id: str = "livox_voxel",
                            roll_deg: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0,
                            tx: float = 0.0, ty: float = 0.0, tz: float = 0.0,
                            scale: float = 1.0):
    a = _get_voxel_adapter(adapter_id)
    a.set_transform({
        "roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg,
        "tx": tx, "ty": ty, "tz": tz, "scale": scale
    })
    return {"ok": True, "sensor_id": adapter_id, "transform": a.transform, "cleared": True}

# --- Clear & decay endpoints ---
@router.post("/clear")
def clear_grid(adapter_id: str = "livox_voxel"):
    a = _get_voxel_adapter(adapter_id)
    a.grid[:] = 0
    a._topdown_dirty = True
    a._last_update_ts = 0.0
    return {"status": "ok"}

@router.get("/decay/config")
def decay_config(adapter_id: str = "livox_voxel"):
    a = _get_voxel_adapter(adapter_id)
    return {
        "enable_decay": bool(a.enable_decay),
        "decay_alpha": float(a.decay_alpha),
        "decay_period_s": float(a.decay_period_s)
    }

@router.post("/decay/config")
def set_decay_config(adapter_id: str = "livox_voxel",
                     enable_decay: Optional[int] = None,
                     decay_alpha: Optional[float] = None,
                     decay_period_s: Optional[float] = None):
    a = _get_voxel_adapter(adapter_id)
    if enable_decay is not None:
        a.enable_decay = bool(int(enable_decay))
    if decay_alpha is not None:
        a.decay_alpha = float(decay_alpha)
    if decay_period_s is not None:
        a.decay_period_s = float(decay_period_s)
    return decay_config(adapter_id)

@router.post("/decay")
def decay_once(adapter_id: str = "livox_voxel",
               alpha: float = 0.95,
               min_strength: int = 0):
    a = _get_voxel_adapter(adapter_id)
    stats = a.decay(alpha=alpha, min_strength=min_strength)
    return {"status": "ok", "alpha": alpha, "min_strength": min_strength, "stats": stats}


@router.get("/grid.binvox", response_class=Response)
def grid_binvox(adapter_id: str = "livox_voxel", version: int = 2):
    a = _get_voxel_adapter(adapter_id)
    g = a.grid  # uint8
    header = (f"#binvox {version}\n"
              f"dim {a.nx} {a.ny} {a.nz}\n"
              f"translate 0 0 0\nscale 1.0\ndata\n").encode("ascii")
    payload = bytearray()
    flat = g.flatten(order="C")
    i = 0
    while i < flat.size:
        val = int(flat[i]); run = 1; j = i + 1
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
    downscale: int = 2,
    # UI-only rotation; e.g., heading_deg=180 makes front appear at top
    heading_deg: float = 0.0
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
        img = Image.fromarray(arr.astype(np.uint8), mode="L")
    elif scale_mode == "equalize":
        arr = np.clip(arr * gain * (255.0 / 31.0), 0.0, 255.0).astype(np.uint8)
        img = Image.fromarray(arr, mode="L")
        try: img = ImageOps.equalize(img)
        except Exception: pass
    else:
        arr = np.clip(arr * gain * (255.0 / 31.0), 0.0, 255.0)
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

    # rotate the image by heading_deg (UI only)
    hdg = float(heading_deg) % 360.0
    if abs(hdg) > 1e-6:
        img = img.rotate(-hdg, resample=Image.BILINEAR, expand=False)

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
            # arr here is already possibly flipped; use its shape for clamp only
            cy_full = (arr.shape[0] - 1) - cy_full
        cx = max(0, min(w - 1, cx_full))
        cy = max(0, min(h - 1, cy_full))
        draw.line([(cx - 6, cy), (cx + 6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy - 6), (cx, cy + 6)], fill=(255, 0, 0), width=2)

    if downscale and int(downscale) > 1:
        factor = int(downscale)
        img = img.resize((max(1, w//factor), max(1, h//factor)), resample=Image.BILINEAR)

    from io import BytesIO
    bio = BytesIO()
    img.save(bio, format="PNG", optimize=True)
    headers = {"X-Width": str(img.size[0]), "X-Height": str(img.size[1]), "X-Voxel-M": str(a.voxel),
               "X-Heading-Deg": str(heading_deg)}
    return Response(content=bio.getvalue(), media_type="image/png", headers=headers)


# --- Filtered local topdown with window toggles & directional colorized rays ---
@router.get("/topdown_filtered.png", response_class=Response)
def topdown_filtered_png(
    adapter_id: str = "livox_voxel",
    radius_m: float = 1.0,
    z_center_m: float = 0.0,
    z_half_thickness_m: float = 1.0,
    forward_only: int = 1,
    window: int = 1,      # 1=use XY window; 0=full XY
    z_window: int = 1,    # 1=use Z slice;  0=full Z
    scale_mode: str = "auto",
    gain: float = 8.0,
    clip_min: int = 0,
    clip_max: int = 31,
    cmap: str = "gray",
    draw_grid: int = 0,           # OFF by default per preference
    tick_m: float = 0.25,
    mark_center: int = 1,
    invert_y: int = 0,
    downscale: int = 1,
    upscale: int = 1,
    upscale_mode: str = "nearest",
    colorize_rays: int = 1,
    rays_n: int = 16,
    rays_steps: int = 8,
    arrow_scale_m: float = 0.5,
    flat_thresh_deg: float = 12.0,
    warn_thresh_deg: float = 25.0,
    climb_thresh_deg: float = 45.0,
    rays_inner_gap_m: float = 3.0,
    heading_deg: float = 0.0
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

    proj = _topdown_from_window(a, x0, x1, y0, y1, z0, z1)
    if invert_y:
        proj = np.flipud(proj)

    arr = proj.astype(np.float32)
    arr = np.clip(arr, float(clip_min), float(clip_max))

    # scale
    if scale_mode == "auto":
        p5, p95 = np.percentile(arr, 5), np.percentile(arr, 95)
        arr = (arr - p5) * (255.0 / max(1e-6, p95 - p5))
        arr = np.clip(arr, 0.0, 255.0); img = Image.fromarray(arr.astype(np.uint8), mode="L")
    elif scale_mode == "equalize":
        arr = np.clip(arr * (255.0 / 31.0) * gain, 0.0, 255.0).astype(np.uint8)
        img = Image.fromarray(arr, mode="L")
        try: img = ImageOps.equalize(img)
        except Exception: pass
    else:
        arr = np.clip(arr * (255.0 / 31.0) * gain, 0.0, 255.0); img = Image.fromarray(arr.astype(np.uint8), mode="L")

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

    # rotate whole image by heading_deg (UI-only)
    hdg = float(heading_deg) % 360.0
    if abs(hdg) > 1e-6:
        img = img.rotate(-hdg, resample=Image.BILINEAR, expand=False)

    draw = ImageDraw.Draw(img)
    h_px, w_px = proj.shape[0], proj.shape[1]  # (height=x, width=y)
    w_img, h_img = img.size

    # optional grid
    if int(draw_grid) == 1:
        spacing_px = max(1, int(round(tick_m / a.voxel)))
        col = (200, 200, 200) if cmap == "gray" else (255, 255, 255)
        for x in range(0, w_img, spacing_px):
            draw.line([(x, 0), (x, h_img - 1)], fill=col, width=1)
        for y in range(0, h_img, spacing_px):
            draw.line([(0, y), (w_img - 1, y)], fill=col, width=1)

    px_x = int(round((0.0 - x0) / a.voxel))
    px_y = int(round((0.0 - y0) / a.voxel))
    px_x = max(0, min(h_px - 1, px_x))
    px_y = max(0, min(w_px - 1, px_y))
    cx = max(0, min(w_img - 1, px_y))
    cy = max(0, min(h_img - 1, px_x))
    if mark_center:
        draw.line([(cx - 6, cy), (cx + 6, cy)], fill=(255, 0, 0), width=2)
        draw.line([(cx, cy - 6), (cx, cy + 6)], fill=(255, 0, 0), width=2)

    # rays with inner gap and heading rotation applied to direction
    if int(colorize_rays) == 1:
        H, xs, ys = _heightmap_from_window(a, x0, x1, y0, y1, z0, z1)
        ray_list = _ray_slopes(a, H, xs, ys, 0.0, 0.0, max(4, int(rays_n)), max(2, int(rays_steps)))

        arrow_len_px = max(1, int(round(arrow_scale_m / a.voxel)))
        gap_px = max(0, int(round(float(rays_inner_gap_m) / a.voxel)))

        hdg_rad = math.radians(hdg)
        for ray in ray_list:
            slope = ray["slope_deg"]
            if slope is None:
                color = (128, 128, 128)
            elif slope <= flat_thresh_deg:
                color = (0, 255, 0)
            elif slope <= warn_thresh_deg:
                color = (255, 255, 0)
            elif slope <= climb_thresh_deg:
                color = (255, 165, 0)
            else:
                color = (255, 0, 0)

            # rotate ray direction by heading
            theta = math.radians(ray["dir_deg"]) + hdg_rad

            dx_unit = math.sin(theta)   # +Y in image columns
            dy_unit = math.cos(theta)   # +X in image rows

            start_x = int(round(cx + dx_unit * gap_px))
            start_y = int(round(cy - dy_unit * gap_px))
            end_x   = int(round(start_x + dx_unit * arrow_len_px))
            end_y   = int(round(start_y - dy_unit * arrow_len_px))

            draw.line([(start_x, start_y), (end_x, end_y)], fill=color, width=2)

    # down/upscale
    if downscale and int(downscale) > 1:
        factor = int(downscale)
        img = img.resize((max(1, w_img//factor), max(1, h_img//factor)), resample=Image.BILINEAR)
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
        "X-Heading-Deg": str(heading_deg),
        "X-Window": json.dumps({"x": [round(x0, 3), round(x1, 3)],
                                "y": [round(y0, 3), round(y1, 3)],
                                "z": [round(z0, 3), round(z1, 3)],
                                "forward_only": bool(int(forward_only)),
                                "window": bool(int(window)),
                                "z_window": bool(int(z_window)),
                                "rays_inner_gap_m": round(float(rays_inner_gap_m), 3)})
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
# Traversability probe + summary
# -------------------------------
def _column_max_z(grid: np.ndarray, ix: int, iy: int,
                  zmin: float, voxel: float,
                  iz0: Optional[int] = None, iz1: Optional[int] = None) -> Optional[float]:
    col = grid[ix, iy, :]
    if iz0 is not None or iz1 is not None:
        nz = col.shape[0]
        s0 = 0 if iz0 is None else max(0, min(nz - 1, iz0))
        s1 = (nz - 1) if iz1 is None else max(0, min(nz - 1, iz1))
        if s1 < s0: s0, s1 = s1, s0
        col = col[s0:s1 + 1]; offset = s0
    else:
        offset = 0
    occ = (col & 0x80) > 0
    if not occ.any():
        return None
    iz = int(np.where(occ)[0].max()) + offset
    return zmin + iz * voxel


def _fit_plane_ls(points_xyz: List[Tuple[float, float, float]]) -> Optional[Tuple[float, float, float]]:
    if len(points_xyz) < 3:
        return None
    A = []; b = []
    for (x, y, z) in points_xyz:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        A.append([x, y, 1.0]); b.append(z)
    if len(A) < 3:
        return None
    A = np.array(A, dtype=np.float64); b = np.array(b, dtype=np.float64)
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        a, by, c = float(sol[0]), float(sol[1]), float(sol[2])
        return (a, by, c)
    except Exception:
        return None


@router.get("/traverse/check", response_class=Response)
def traverse_check(adapter_id: str = "livox_voxel",
                   ahead_m: float = 1.0, width_m: float = 1.0, step_limit_m: float = 0.1068,
                   local_radius_m: float = 1.0, z_center_m: float = 0.0, z_half_thickness_m: float = 1.0,
                   forward_only: int = 1, window: int = 1, z_window: int = 1,
                   method: str = "column", rays_n: int = 16, rays_steps: int = 8,
                   cliff_threshold_deg: float = 45.0, debug: int = 0):
    a = _get_voxel_adapter(adapter_id); g = a.grid; voxel = a.voxel

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
        zc = float(z_center_m); zh = max(0.01, float(z_half_thickness_m))
        iz0 = int(max(0, math.floor(((zc - zh) - a.zmin) / voxel)))
        iz1 = int(min(a.nz - 1, math.ceil(((zc + zh) - a.zmin) / voxel)))
    else:
        iz0, iz1 = 0, a.nz - 1

    xs: List[float] = []; hs: List[float] = []; pts3d: List[Tuple[float, float, float]] = []

    for ix in range(ix_start, ix_end + 1):
        h_col = []
        for iy in range(iy_start, iy_end + 1):
            hz = _column_max_z(g, ix, iy, a.zmin, voxel, iz0=iz0, iz1=iz1)
            if hz is not None:
                h_col.append(hz)
                if method == "plane":
                    xw = a.xmin + ix * voxel
                    yw = a.ymin + iy * voxel
                    pts3d.append((xw, yw, hz))
        if h_col:
            xs.append(a.xmin + ix * voxel)
            hs.append(max(h_col))

    if len(hs) < 3:
        out = {"status": "failed", "pitch_deg": None, "max_step_m": None,
               "ok_traverse": False, "ok_cliff": False, "note": "no occupancy in selected window",
               "window": bool(int(window)), "z_window": bool(int(z_window)),
               "forward_only": bool(int(forward_only)), "method": method}
        return Response(content=json.dumps(out), media_type="application/json")

    x_span = (xs[-1] - xs[0]) if len(xs) > 1 else voxel
    dz = (hs[-1] - hs[0])
    pitch_deg_col = math.degrees(math.atan2(dz, max(x_span, 1e-3)))
    max_step = max(abs(hs[i] - hs[i-1]) for i in range(1, len(hs))) if len(hs) > 1 else 0.0

    pitch_deg_used = pitch_deg_col
    slope_info = None
    if method.lower() == "plane":
        coeffs = _fit_plane_ls(pts3d)
        if coeffs is not None:
            ax, by, c = coeffs
            slope = math.sqrt(ax*ax + by*by)
            pitch_deg_plane = math.degrees(math.atan(slope))
            dir_rad = math.atan2(by, ax)
            dir_deg = math.degrees(dir_rad)
            n_raw = np.array([-ax, -by, 1.0], dtype=np.float64)
            n_norm = n_raw / max(1e-9, np.linalg.norm(n_raw))
            slope_info = {"pitch_deg": round(pitch_deg_plane, 2),
                          "dir_deg": round(dir_deg, 2),
                          "normal": [round(float(n_norm[0]),4), round(float(n_norm[1]),4), round(float(n_norm[2]),4)]}
            pitch_deg_used = pitch_deg_plane if pitch_deg_col >= 0 else -pitch_deg_plane

    ok_traverse = (abs(pitch_deg_used) <= a.climb_limit_deg) and (max_step <= step_limit_m)

    # directional rays for cliff
    x0w = a.xmin + ix_start * voxel; x1w = a.xmin + ix_end * voxel
    y0w = a.ymin + iy_start * voxel; y1w = a.ymin + iy_end * voxel
    z0w = a.zmin + iz0 * voxel;      z1w = a.zmin + iz1 * voxel
    H, xcoords, ycoords = _heightmap_from_window(a, x0w, x1w, y0w, y1w, z0w, z1w)

    rays = _ray_slopes(a, H, xcoords, ycoords, 0.0, 0.0, max(4, int(rays_n)), max(2, int(rays_steps)))
    ray_slopes = [r["slope_deg"] for r in rays if r["slope_deg"] is not None]
    if len(ray_slopes) == 0:
        cliff_max = None; cliff_dir = None; ok_cliff = False; status = "failed"
    else:
        idx = int(np.nanargmax(ray_slopes))
        cliff_max = float(ray_slopes[idx])
        cliff_dir = float(rays[idx]["dir_deg"])
        ok_cliff = (cliff_max <= float(cliff_threshold_deg))
        status = "ok"

    out = {
        "status": status,
        "pitch_deg": round(pitch_deg_used, 2),
        "pitch_deg_raw_column": round(pitch_deg_col, 2),
        "max_step_m": round(max_step, 3),
        "ok_traverse": bool(ok_traverse),
        "ok_cliff": bool(ok_cliff),
        "climb_limit_deg": a.climb_limit_deg,
        "step_limit_m": step_limit_m,
        "cliff_threshold_deg": cliff_threshold_deg,
        "cliff_max_deg": None if cliff_max is None else round(cliff_max, 2),
        "cliff_dir_deg": None if cliff_dir is None else round(cliff_dir, 2),
        "forward_only": bool(int(forward_only)),
        "window": bool(int(window)),
        "z_window": bool(int(z_window)),
        "method": method,
        "x_window_m": [round(a.xmin + ix_start * voxel, 3), round(a.xmin + ix_end * voxel, 3)],
        "y_window_m": [round(a.ymin + iy_start * voxel, 3), round(a.ymin + iy_end * voxel, 3)],
        "z_slice_m": [round(a.zmin + iz0 * voxel, 3), round(a.zmin + iz1 * voxel, 3)]
    }
    if slope_info:
        out["slope"] = slope_info
    if debug == 1:
        out["rays"] = rays
    return Response(content=json.dumps(out), media_type="application/json")


@router.get("/traverse/summary", response_class=Response)
def traverse_summary(adapter_id: str = "livox_voxel",
                     ahead_m: float = 1.0, width_m: float = 1.0, local_radius_m: float = 1.0,
                     z_center_m: float = 0.0, z_half_thickness_m: float = 1.0,
                     forward_only: int = 1, window: int = 1, z_window: int = 1,
                     method: str = "plane", rays_n: int = 16, rays_steps: int = 8,
                     cliff_threshold_deg: float = 45.0,
                     img_radius_m: float = 1.0, img_forward_only: int = 1,
                     img_window: int = 1, img_z_window: int = 1, img_upscale: int = 4,
                     img_cmap: str = "gray", img_flat_thresh_deg: float = 12.0,
                     img_warn_thresh_deg: float = 25.0, img_climb_thresh_deg: float = 45.0,
                     heading_deg: float = 0.0):
    # reuse check
    res = traverse_check(adapter_id=adapter_id, ahead_m=ahead_m, width_m=width_m,
                         step_limit_m=0.1068, local_radius_m=local_radius_m,
                         z_center_m=z_center_m, z_half_thickness_m=z_half_thickness_m,
                         forward_only=forward_only, window=window, z_window=z_window,
                         method=method, rays_n=rays_n, rays_steps=rays_steps,
                         cliff_threshold_deg=cliff_threshold_deg, debug=0)
    data = json.loads(res.body.decode("utf-8")) if hasattr(res, "body") else {}

    # compose image URL
    params = {
        "radius_m": img_radius_m,
        "forward_only": img_forward_only,
        "window": img_window,
        "z_window": img_z_window,
        "colorize_rays": 1,
        "rays_n": rays_n,
        "rays_steps": rays_steps,
        "flat_thresh_deg": img_flat_thresh_deg,
        "warn_thresh_deg": img_warn_thresh_deg,
        "climb_thresh_deg": img_climb_thresh_deg,
        "upscale": img_upscale,
        "cmap": img_cmap,
        "draw_grid": 0,               # preference: grid off
        "rays_inner_gap_m": 3.0,
        "heading_deg": heading_deg
    }
    q = "&".join([f"{k}={json.dumps(v) if isinstance(v,(dict,list)) else v}" for k, v in params.items()])
    img_url = f"/livox_voxel/topdown_filtered.png?{q}"

    summary = {
        "status": data.get("status", "ok"),
        "ok_traverse": bool(data.get("ok_traverse", False)),
        "ok_cliff": bool(data.get("ok_cliff", False)),
        "pitch_deg": data.get("pitch_deg"),
        "max_step_m": data.get("max_step_m"),
        "climb_limit_deg": data.get("climb_limit_deg"),
        "step_limit_m": data.get("step_limit_m"),
        "cliff": {
            "threshold_deg": cliff_threshold_deg,
            "max_deg": data.get("cliff_max_deg"),
            "dir_deg": data.get("cliff_dir_deg")
        },
        "window": {
            "forward_only": bool(int(forward_only)),
            "xy": bool(int(window)),
            "z": bool(int(z_window)),
            "x": data.get("x_window_m"),
            "y": data.get("y_window_m"),
            "z_slice": data.get("z_slice_m")
        },
        "image": {"url": img_url}
    }
    return Response(content=json.dumps(summary), media_type="application/json")


# --- Window helpers and ray slopes (used by filtered PNG & traverse) ---
def _clamped_window_indices(a: LivoxVoxelAdapter,
                            x_min: float, x_max: float,
                            y_min: float, y_max: float,
                            z_min: float, z_max: float) -> Tuple[int,int,int,int,int,int]:
    x_min = max(a.xmin, x_min); x_max = min(a.xmax, x_max)
    y_min = max(a.ymin, y_min); y_max = min(a.ymax, y_max)
    z_min = max(a.zmin, z_min); z_max = min(a.zmax, z_max)
    if x_min >= x_max or y_min >= y_max or z_min >= z_max:
        return 0, -1, 0, -1, 0, -1
    ix0 = int((x_min - a.xmin) / a.voxel)
    ix1 = int((x_max - a.xmin) / a.voxel)
    iy0 = int((y_min - a.ymin) / a.voxel)
    iy1 = int((y_max - a.ymin) / a.voxel)
    iz0 = int((z_min - a.zmin) / a.voxel)
    iz1 = int((z_max - a.zmin) / a.voxel)
    ix1 = min(ix1, a.nx - 1)
    iy1 = min(iy1, a.ny - 1)
    iz1 = min(iz1, a.nz - 1)
    return ix0, ix1, iy0, iy1, iz0, iz1


def _topdown_from_window(a: LivoxVoxelAdapter,
                         x_min: float, x_max: float,
                         y_min: float, y_max: float,
                         z_min: float, z_max: float) -> np.ndarray:
    ix0, ix1, iy0, iy1, iz0, iz1 = _clamped_window_indices(a, x_min, x_max, y_min, y_max, z_min, z_max)
    if ix1 < ix0 or iy1 < iy0 or iz1 < iz0:
        return np.zeros((1, 1), dtype=np.uint8)
    g = a.grid[ix0:ix1+1, iy0:iy1+1, iz0:iz1+1]
    occ = (g & 0x80) > 0
    strength = (g & 0x1F)
    proj = np.where(occ.any(axis=2), strength.max(axis=2), 0).astype(np.uint8)
    return proj


def _heightmap_from_window(a: LivoxVoxelAdapter,
                           x_min: float, x_max: float,
                           y_min: float, y_max: float,
                           z_min: float, z_max: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    ix0, ix1, iy0, iy1, iz0, iz1 = _clamped_window_indices(a, x_min, x_max, y_min, y_max, z_min, z_max)
    if ix1 < ix0 or iy1 < iy0 or iz1 < iz0:
        return np.full((1,1), np.nan, dtype=np.float32), np.array([0.0]), np.array([0.0])

    g = a.grid[ix0:ix1+1, iy0:iy1+1, iz0:iz1+1]
    occ = (g & 0x80) > 0
    H = np.full((g.shape[0], g.shape[1]), np.nan, dtype=np.float32)
    any_occ = occ.any(axis=2)
    where_any = np.where(any_occ)
    if where_any[0].size > 0:
        # Highest occupied z per column
        occ_rev = occ[:, :, ::-1]
        iz_rev = np.argmax(occ_rev, axis=2)
        iz_top = occ.shape[2] - 1 - iz_rev
        H[any_occ] = (a.zmin + (iz0 + iz_top[any_occ]) * a.voxel).astype(np.float32)

    xs = a.xmin + (np.arange(ix0, ix1+1) * a.voxel)
    ys = a.ymin + (np.arange(iy0, iy1+1) * a.voxel)
    return H, xs, ys


def _ray_slopes(a: LivoxVoxelAdapter, H: np.ndarray, xs: np.ndarray, ys: np.ndarray,
                x0: float, y0: float, n_dirs: int, steps: int) -> List[Dict[str, float]]:
    ix_c = int(round((x0 - xs[0]) / a.voxel))
    iy_c = int(round((y0 - ys[0]) / a.voxel))
    ix_c = max(0, min(H.shape[0] - 1, ix_c))
    iy_c = max(0, min(H.shape[1] - 1, iy_c))
    out: List[Dict[str, float]] = []
    for k in range(n_dirs):
        theta = (k / n_dirs) * 2.0 * math.pi
        dx = math.cos(theta); dy = math.sin(theta)
        z_start = H[ix_c, iy_c] if math.isfinite(H[ix_c, iy_c]) else np.nan
        z_end = np.nan; run_m = 0.0
        ix, iy = ix_c, iy_c
        for s in range(1, max(1, steps) + 1):
            ix = int(round(ix_c + dx * s))
            iy = int(round(iy_c + dy * s))
            if ix < 0 or iy < 0 or ix >= H.shape[0] or iy >= H.shape[1]:
                break
            z = H[ix, iy]
            if math.isfinite(z):
                z_end = z
                run_m = math.sqrt(((xs[ix] - xs[ix_c])**2) + ((ys[iy] - ys[iy_c])**2))
                break
        if not math.isfinite(z_start) or not math.isfinite(z_end) or run_m <= 1e-6:
            slope = None
        else:
            dz = float(z_end - z_start)
            slope = math.degrees(math.atan2(abs(dz), max(run_m, 1e-6)))
        out.append({"dir_deg": math.degrees(theta), "slope_deg": None if slope is None else round(slope, 2)})
    return out

# Bind helpers to adapter for reuse
LivoxVoxelAdapter._topdown_from_window = lambda self, x0,x1,y0,y1,z0,z1: _topdown_from_window(self, x0,x1,y0,y1,z0,z1)
LivoxVoxelAdapter._heightmap_from_window = lambda self, x0,x1,y0,y1,z0,z1: _heightmap_from_window(self, x0,x1,y0,y1,z0,z1)
LivoxVoxelAdapter._ray_slopes = lambda self, H,xs,ys,x0,y0,n_dirs,steps: _ray_slopes(self, H,xs,ys,x0,y0,n_dirs,steps)

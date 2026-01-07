
# src/sensorhub/main.py
import os
import math
import traceback
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.gzip import GZipMiddleware

# Prefer ORJSONResponse if available (auto-fallback if not)
try:
    from fastapi.responses import ORJSONResponse as DefaultResponse
except Exception:
    from fastapi.responses import JSONResponse as DefaultResponse

from .api.routes import router as sensors_router
from .api.ws import router as ws_router
from .core.sensor_manager import manager
from .logging_config import configure_logging
from .api.health import router as health_router
from .api.video import router as video_router
from sensorhub.adapters.livox_mid360.livox_adapter import router as livox_router
from sensorhub.api.snapshot import router as snapshot_router
from sensorhub.api.livox_snapshot import router as livox_snapshot_router

# -------- logging --------
configure_logging()
import logging
_rplidar_log_level = os.getenv("RPLIDAR_LOG_LEVEL", "INFO").upper()
logging.getLogger("sensorhub.adapters.rplidar_s2").setLevel(_rplidar_log_level)
logging.getLogger("sensorhub.adapters.rplidar_s2.rplidar_adapter").setLevel(_rplidar_log_level)

# -------- app --------
app = FastAPI(
    title="SensorHub",
    description="Modular API/WebSocket service for robot sensors (with USB camera streaming)",
    version="0.3.2",
    default_response_class=DefaultResponse,
)

# CORS
allow_origins = os.getenv("SENSORHUB_CORS", "*").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=allow_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# GZip for payloads >= 1KB
app.add_middleware(GZipMiddleware, minimum_size=1024)

# Routers
app.include_router(health_router)
app.include_router(sensors_router)
app.include_router(video_router)
app.include_router(ws_router)
app.include_router(livox_router)
app.include_router(snapshot_router)
app.include_router(livox_snapshot_router)

# -------- enhanced /sensors & health endpoints --------
@app.get("/sensors")
def list_sensors():
    """
    Return all registered sensors with status/health fields.
    Supersedes the minimal list from .api.routes.
    """
    infos = manager.list_with_status()
    # Pydantic BaseModel not guaranteed; ensure plain dicts
    return infos

@app.get("/sensors/{sensor_id}/health")
def sensor_health(sensor_id: str):
    """
    Health endpoint for a specific adapter.
    """
    h = manager.get_status(sensor_id)
    if h is None:
        raise HTTPException(status_code=404, detail="sensor not found")
    return h

# -------- helpers --------
def _get_adapter_by_id(sensor_id: str):
    # Prefer manager.get_adapter()
    try:
        a = manager.get_adapter(sensor_id)
        if a:
            return a
    except Exception:
        pass
    # Fallbacks for older setups
    for meth in ("get_adapter", "get_sensor", "get"):
        if hasattr(manager, meth):
            try:
                a = getattr(manager, meth)(sensor_id)
                if a:
                    return a
            except Exception:
                pass
    for attr in ("adapters", "_adapters", "sensors", "_registry"):
        if hasattr(manager, attr):
            reg = getattr(manager, attr)
            if isinstance(reg, dict):
                a = reg.get(sensor_id)
                if a:
                    return a
            elif isinstance(reg, (list, tuple)):
                for a in reg:
                    if getattr(a, "sensor_id", None) == sensor_id:
                        return a
    return None

def _filter_points(a: List[float], r: List[float], q: List[int], min_q: int = 1, min_r_mm: int = 1):
    ao, ro, qo = [], [], []
    for ai, ri, qi in zip(a, r, q):
        if qi is None or ri is None:
            continue
        if qi >= min_q and ri >= min_r_mm:
            ao.append(float(ai)); ro.append(float(ri)); qo.append(int(qi))
    return ao, ro, qo

def _decimate(a: List[float], r: List[float], q: List[int], max_points: int):
    n = len(a)
    if max_points <= 0 or n <= max_points:
        return a, r, q
    step = max(1, n // max_points)
    return a[::step], r[::step], q[::step]

def _polar_to_xy_m(a: List[float], r: List[float]):
    xs, ys = [], []
    for ai, ri in zip(a, r):
        rad = math.radians(ai); m = ri / 1000.0
        xs.append(m * math.cos(rad)); ys.append(m * math.sin(rad))
    return xs, ys

def _round(values: List[float], decimals: int):
    if decimals is None or decimals < 0:
        return values
    return [round(v, decimals) for v in values]

# -------- endpoints --------

@app.get("/sensors/{sensor_id}/latest_raw")
def get_latest_raw(
    sensor_id: str,
    max_points: int = Query(4096, ge=1, le=65536),
    decimals: int = Query(2, ge=0, le=6),
    include_meta: bool = True,
):
    """
    Return the adapter's raw cached points for the last published revolution.
    Falls back to the latest Sample's points if the adapter cache is empty.
    """
    adapter = _get_adapter_by_id(sensor_id)
    if not adapter:
        raise HTTPException(status_code=404, detail="sensor not found")

    # Try adapter raw cache first (method or property)
    points: Optional[List[Dict[str, Any]]] = None
    raw_accessor = getattr(adapter, "latest_raw_points", None)
    try:
        if callable(raw_accessor):
            points = raw_accessor()
        elif raw_accessor is not None:
            points = raw_accessor
    except Exception:
        points = None

    # Fallback to manager.latest() if cache empty
    if not points:
        sample = manager.latest(sensor_id)
        if not sample or not isinstance(sample.data, dict):
            raise HTTPException(status_code=404, detail="sensor or sample not found")
        points = sample.data.get("points") or []
    if not points:
        # No points yet; return 204 No Content with informative detail
        raise HTTPException(status_code=204, detail="no points in cache or latest sample")

    # Clamp and format decimals
    pts = points[:max_points]
    if decimals > 0:
        for p in pts:
            try:
                p["angle_deg"] = round(float(p["angle_deg"]), decimals)
            except Exception:
                pass
            try:
                p["distance_mm"] = round(float(p["distance_mm"]), decimals)
            except Exception:
                pass
            # quality remains int

    out: Dict[str, Any] = {
        "id": sensor_id,
        "kind": getattr(adapter, "kind", None),
        "count": len(pts),
        "points": pts,
    }
    if include_meta:
        h = adapter.health()
        out["meta"] = {"ts": h.get("last_sample_ts"), "status": h.get("status")}
    return out

@app.get("/sensors/{sensor_id}/latest")
def get_latest(
    sensor_id: str,
    include_points: bool = Query(True),
    max_points: int = Query(4096, ge=1),
    keep: str = Query("raw", pattern="^(raw|xy)$"),  # <-- fixed regex
    decimals: int = Query(2, ge=0, le=6),
    include_meta: bool = Query(True),
    filter_invalid: bool = Query(True),
    min_quality: int = Query(1, ge=0),
    min_range_mm: int = Query(1, ge=0),
):
    """
    Return the latest Sample from the manager. If it contains 'points', clamp to max_points.
    Backward-compatible XY formatting retained if 'keep=xy' and angles/ranges are present.
    """
    sample = manager.latest(sensor_id)
    if not sample:
        raise HTTPException(status_code=404, detail="sensor or sample not found")

    data = sample.data if isinstance(sample.data, dict) else {}
    resp: Dict[str, Any] = {}

    if include_meta:
        resp["sensor_id"] = sample.sensor_id
        # 'sample.ts' is datetime; use ISO string for JSON
        ts = getattr(sample, "ts", None)
        resp["ts"] = ts.isoformat().replace("+00:00", "Z") if ts else None

    # Preferred path: points array produced by adapter
    if include_points and "points" in data and isinstance(data["points"], list):
        points = data["points"][:max_points]
        # Optionally format decimals
        if decimals > 0:
            for p in points:
                try:
                    p["angle_deg"] = round(float(p["angle_deg"]), decimals)
                except Exception:
                    pass
                try:
                    p["distance_mm"] = round(float(p["distance_mm"]), decimals)
                except Exception:
                    pass
        resp["data"] = {"points": points}
        return resp

    # Backward-compatible path for legacy frame structure (angles/ranges/qualities)
    angles = list(data.get("angles", []))
    ranges = list(data.get("ranges", []))
    qualities = list(data.get("qualities", []))

    if include_points and not (len(angles) == len(ranges) == len(qualities)):
        n = min(len(angles), len(ranges), len(qualities))
        angles, ranges, qualities = angles[:n], ranges[:n], qualities[:n]

    if include_points and filter_invalid:
        angles, ranges, qualities = _filter_points(
            angles, ranges, qualities, min_q=min_quality, min_r_mm=min_range_mm
        )

    if include_points:
        angles, ranges, qualities = _decimate(angles, ranges, qualities, max_points)

    # Build legacy response
    legacy = {}
    if keep == "xy":
        x_m, y_m = _polar_to_xy_m(angles, ranges)
        legacy["x"] = _round(x_m, decimals)
        legacy["y"] = _round(y_m, decimals)
        legacy["qualities"] = qualities
    else:
        legacy["angles"] = _round(angles, decimals)
        legacy["ranges"] = _round(ranges, decimals)
        legacy["qualities"] = qualities

    resp["data"] = legacy
    return resp

@app.on_event("startup")
async def startup_event():
    cfg_path = os.getenv(
        "SENSORHUB_CONFIG",
        str(Path(__file__).parent / "config" / "config.video.yaml"),
    )
    print(f"[SensorHub] Loading sensors from config: {cfg_path}")
    try:
        manager.load_from_config(Path(cfg_path))
    except Exception as e:
        print("[SensorHub] ERROR loading config:")
        print(f" Path: {cfg_path}")
        print(f" Exception: {e.__class__.__name__}: {e}")
        print(" Traceback:")
        traceback.print_exc()
        raise

@app.get("/")
def root():
    return {"ok": True, "service": "SensorHub", "version": "0.3.2"}

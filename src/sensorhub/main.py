
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

# ---------- logging ----------
configure_logging()
import logging
_rplidar_log_level = os.getenv("RPLIDAR_LOG_LEVEL", "INFO").upper()
logging.getLogger("sensorhub.adapters.rplidar_s2").setLevel(_rplidar_log_level)
logging.getLogger("sensorhub.adapters.rplidar_s2.rplidar_adapter").setLevel(_rplidar_log_level)

# ---------- app ----------
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

# ---------- helpers ----------
def _get_adapter_by_id(sensor_id: str):
    for meth in ("get_adapter", "get_sensor", "get"):
        if hasattr(manager, meth):
            try:
                a = getattr(manager, meth)(sensor_id)
                if a: return a
            except Exception:
                pass
    for attr in ("adapters", "_adapters", "sensors", "_registry"):
        if hasattr(manager, attr):
            reg = getattr(manager, attr)
            if isinstance(reg, dict):
                a = reg.get(sensor_id)
                if a: return a
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
    if max_points <= 0 or n <= max_points: return a, r, q
    step = max(1, n // max_points)
    return a[::step], r[::step], q[::step]

def _polar_to_xy_m(a: List[float], r: List[float]):
    xs, ys = [], []
    for ai, ri in zip(a, r):
        rad = math.radians(ai); m = ri / 1000.0
        xs.append(m * math.cos(rad)); ys.append(m * math.sin(rad))
    return xs, ys

def _round(values: List[float], decimals: int):
    if decimals is None or decimals < 0: return values
    return [round(v, decimals) for v in values]

def _get_latest_frame(sensor_id: str) -> Optional[Dict[str, Any]]:
    frame: Optional[Dict[str, Any]] = None
    if hasattr(manager, "get_latest_sample"):
        try: frame = manager.get_latest_sample(sensor_id)
        except Exception: frame = None
    elif hasattr(manager, "latest_samples"):
        try: frame = manager.latest_samples.get(sensor_id)  # type: ignore
        except Exception: frame = None
    if frame is None:
        adapter = _get_adapter_by_id(sensor_id)
        if adapter and hasattr(adapter, "get_latest_frame"):
            try: frame = adapter.get_latest_frame()
            except Exception: frame = None
    return frame

# ---------- endpoints ----------
@app.get("/sensors/{sensor_id}/latest_raw")
def get_latest_raw(sensor_id: str):
    adapter = _get_adapter_by_id(sensor_id)
    if not adapter or not hasattr(adapter, "get_latest_frame"):
        raise HTTPException(status_code=404, detail="sensor or sample not found")
    frame = adapter.get_latest_frame()
    if frame is None:
        raise HTTPException(status_code=204, detail="no frame yet")
    return frame

@app.get("/sensors/{sensor_id}/latest")
def get_latest(
    sensor_id: str,
    include_points: bool = Query(True),
    max_points: int = Query(20000, ge=1),
    keep: str = Query("raw", pattern="^(raw|xy)$"),  # <-- fixed regex
    decimals: int = Query(2, ge=0, le=6),
    include_meta: bool = Query(True),
    filter_invalid: bool = Query(True),
    min_quality: int = Query(1, ge=0),
    min_range_mm: int = Query(1, ge=0),
):
    frame = _get_latest_frame(sensor_id)
    if frame is None:
        raise HTTPException(status_code=404, detail="sensor or sample not found")

    angles = list(frame.get("angles", []))
    ranges = list(frame.get("ranges", []))
    qualities = list(frame.get("qualities", []))

    if include_points and not (len(angles) == len(ranges) == len(qualities)):
        n = min(len(angles), len(ranges), len(qualities))
        angles, ranges, qualities = angles[:n], ranges[:n], qualities[:n]

    if include_points and filter_invalid:
        angles, ranges, qualities = _filter_points(
            angles, ranges, qualities, min_q=min_quality, min_r_mm=min_range_mm
        )

    if include_points:
        angles, ranges, qualities = _decimate(angles, ranges, qualities, max_points)

    resp: Dict[str, Any] = {}
    count = len(angles)

    if include_meta:
        resp["sensor_id"] = sensor_id
        resp["timestamp"] = frame.get("timestamp")
        resp["partial"] = bool(frame.get("partial", False))
        resp["count"] = count

    if include_points:
        if keep == "xy":
            x_m, y_m = _polar_to_xy_m(angles, ranges)
            resp["x"] = _round(x_m, decimals)
            resp["y"] = _round(y_m, decimals)
            resp["qualities"] = qualities
        else:
            resp["angles"] = _round(angles, decimals)
            resp["ranges"] = _round(ranges, decimals)
            resp["qualities"] = qualities

    if not include_points and not include_meta:
        resp = {"ok": True, "sensor_id": sensor_id, "count": count}

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


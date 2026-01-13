
# src/sensorhub/main.py
import os
import math
import traceback
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

from fastapi import FastAPI, HTTPException, Query, Body
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
from .api.summary import router as summary_router
from sensorhub.adapters.livox_mid360.livox_adapter import router as livox_router
from sensorhub.api.snapshot import router as snapshot_router
from sensorhub.api.livox_snapshot import router as livox_snapshot_router
from sensorhub.api.sensors_latest import router as sensors_latest_router
from fastapi.staticfiles import StaticFiles

# NEW: voxel router
from sensorhub.adapters.livox_mid360.livox_voxel_adapter import router as livox_voxel_router

# Add Prometheus instrumentation
from prometheus_fastapi_instrumentator import Instrumentator

# NEW: transform helpers
try:
    from sensorhub.core.transform import Transform, apply_transform_xyz
except Exception:
    Transform = None  # type: ignore
    def apply_transform_xyz(x, y, z, t):  # type: ignore
        return x, y, z

# ---------------- logging ----------------
configure_logging()
import logging
_rplidar_log_level = os.getenv("RPLIDAR_LOG_LEVEL", "INFO").upper()
logging.getLogger("sensorhub.adapters.rplidar_s2").setLevel(_rplidar_log_level)
logging.getLogger("sensorhub.adapters.rplidar_s2.rplidar_adapter").setLevel(_rplidar_log_level)

# ---------------- app ----------------
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
app.include_router(livox_snapshot_router)
app.include_router(snapshot_router)
app.include_router(summary_router)
app.include_router(sensors_latest_router)

# NEW: include voxel router
app.include_router(livox_voxel_router)

static_dir = Path(__file__).resolve().parent / "static"
app.mount("/ui", StaticFiles(directory=str(static_dir), html=True))

Instrumentator().instrument(app).expose(app, endpoint="/metrics", include_in_schema=False)

# ---------------- enhanced /sensors & health endpoints ----------------
@app.get("/sensors")
def list_sensors():
    infos = manager.list_with_status()
    return infos

@app.get("/sensors/{sensor_id}/health")
def sensor_health(sensor_id: str):
    h = manager.get_status(sensor_id)
    if h is None:
        raise HTTPException(status_code=404, detail="sensor not found")
    return h

# ---------------- helpers ----------------
def _get_adapter_by_id(sensor_id: str):
    try:
        a = manager.get_adapter(sensor_id)
        if a:
            return a
    except Exception:
        pass
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

# ---------------- existing endpoints (raw & latest) ----------------
@app.get("/sensors/{sensor_id}/latest_raw")
def get_latest_raw(
    sensor_id: str,
    max_points: int = Query(4096, ge=1, le=65536),
    decimals: int = Query(2, ge=0, le=6),
    include_meta: bool = True,
):
    adapter = _get_adapter_by_id(sensor_id)
    if not adapter:
        raise HTTPException(status_code=404, detail="sensor not found")

    points: Optional[List[Dict[str, Any]]] = None
    raw_accessor = getattr(adapter, "latest_raw_points", None)
    try:
        if callable(raw_accessor):
            points = raw_accessor()
        elif raw_accessor is not None:
            points = raw_accessor
    except Exception:
        points = None

    if not points:
        sample = manager.latest(sensor_id)
        if not sample or not isinstance(sample.data, dict):
            raise HTTPException(status_code=404, detail="sensor or sample not found")
        points = sample.data.get("points") or []
    if not points:
        raise HTTPException(status_code=204, detail="no points in cache or latest sample")

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

    out: Dict[str, Any] = {
        "id": sensor_id,
        "kind": getattr(adapter, "kind", None),
        "frame": "sensor",
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
    keep: str = Query("raw", pattern="^(raw|xy)$"),
    decimals: int = Query(2, ge=0, le=6),
    include_meta: bool = Query(True),
    filter_invalid: bool = Query(True),
    min_quality: int = Query(1, ge=0),
    min_range_mm: int = Query(1, ge=0),
):
    sample = manager.latest(sensor_id)
    if not sample:
        raise HTTPException(status_code=404, detail="sensor or sample not found")

    data = sample.data if isinstance(sample.data, dict) else {}
    resp: Dict[str, Any] = {}

    if include_meta:
        resp["sensor_id"] = sample.sensor_id
        ts = getattr(sample, "ts", None)
        resp["ts"] = ts.isoformat().replace("+00:00", "Z") if ts else None
        resp["frame"] = "sensor"

    if include_points and "points" in data and isinstance(data["points"], list):
        points = data["points"][:max_points]
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

# ---------------- STARTUP ----------------
CONFIG_PATH: Optional[Path] = None

@app.on_event("startup")
async def startup_event():
    global CONFIG_PATH
    cfg_path = os.getenv(
        "SENSORHUB_CONFIG",
        str(Path(__file__).parent / "config" / "config.video.yaml"),
    )
    CONFIG_PATH = Path(cfg_path)

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


static_dir = Path(__file__).resolve().parent / "static"
app.mount("/ui", StaticFiles(directory=str(static_dir), html=True))

from fastapi.responses import RedirectResponse
@app.get("/", include_in_schema=False)
def gateway_root():
    return RedirectResponse(url="/ui/index.html", status_code=307)

# ---------------- TRANSFORM API ----------------
def _update_transform_in_config_yaml(sensor_id: str, t: Transform, unity_euler_deg: Optional[Dict[str, float]] = None) -> None:
    """
    Update the 'transform' block (and optional 'unity_euler_deg') for the given sensor in the CONFIG_PATH YAML.
    """
    if CONFIG_PATH is None or not CONFIG_PATH.exists():
        raise RuntimeError("CONFIG_PATH not set or does not exist")

    import yaml
    from fastapi.responses import FileResponse
    cfg = yaml.safe_load(CONFIG_PATH.read_text()) or {}
    sensors = cfg.get("sensors", [])
    updated = False
    for entry in sensors:
        if str(entry.get("id")) == sensor_id:
            params = entry.get("params", {}) or {}
            params["transform"] = t.to_dict() if hasattr(t, "to_dict") else t  # type: ignore
            if unity_euler_deg and isinstance(unity_euler_deg, dict):
                params["unity_euler_deg"] = {
                    "x": float(unity_euler_deg.get("x", 0.0)),
                    "y": float(unity_euler_deg.get("y", 0.0)),
                    "z": float(unity_euler_deg.get("z", 0.0)),
                }
            entry["params"] = params
            updated = True
            break
    if not updated:
        raise RuntimeError(f"sensor id '{sensor_id}' not found in config")

    CONFIG_PATH.write_text(yaml.safe_dump(cfg, sort_keys=False))

@app.get("/sensors/{sensor_id}/transform")
def get_transform(sensor_id: str):
    adapter = _get_adapter_by_id(sensor_id)
    if not adapter:
        raise HTTPException(status_code=404, detail="sensor not found")
    # robot transform
    t = adapter.get_transform()
    out = {"sensor_id": sensor_id, "transform": t.to_dict() if hasattr(t, "to_dict") else t}  # type: ignore
    # unity euler override (display only)
    ue = getattr(adapter, "unity_euler_deg", None)
    if ue:
        out["unity_euler_deg"] = {"x": float(ue.get("x", 0.0)), "y": float(ue.get("y", 0.0)), "z": float(ue.get("z", 0.0))}
    return out

@app.put("/sensors/{sensor_id}/transform")
def put_transform(
    sensor_id: str,
    body: Dict[str, Any] = Body(..., example={
        "tx": 0.0, "ty": 0.0, "tz": 0.0,
        "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
        "scale": 1.0,
        "unity_euler_deg": {"x": 0.0, "y": 0.0, "z": 0.0}
    })
):
    adapter = _get_adapter_by_id(sensor_id)
    if not adapter:
        raise HTTPException(status_code=404, detail="sensor not found")

    # robot transform (required keys live in body; tolerate extra keys)
    t = Transform.from_dict(body or {}) if Transform is not None else body  # type: ignore
    # set in-memory
    adapter.set_transform(t if Transform is None else t)  # type: ignore

    # optional unity_euler_deg override
    ue_in = body.get("unity_euler_deg")
    if ue_in and isinstance(ue_in, dict):
        setattr(adapter, "unity_euler_deg", {
            "x": float(ue_in.get("x", 0.0)),
            "y": float(ue_in.get("y", 0.0)),
            "z": float(ue_in.get("z", 0.0)),
        })

    # persist to YAML
    try:
        _update_transform_in_config_yaml(sensor_id, t if Transform is None else t, getattr(adapter, "unity_euler_deg", None))  # type: ignore
    except Exception as e:
        return {"ok": True, "sensor_id": sensor_id, "transform": getattr(t, "to_dict", lambda: t)(), "unity_euler_deg": getattr(adapter, "unity_euler_deg", None), "persist_warning": str(e)}  # type: ignore

    return {"ok": True, "sensor_id": sensor_id, "transform": getattr(t, "to_dict", lambda: t)(), "unity_euler_deg": getattr(adapter, "unity_euler_deg", None)}  # type: ignore

# ---------------- TRANSFORMED POINTS ----------------
@app.get("/sensors/{sensor_id}/latest_transformed")
def get_latest_transformed(
    sensor_id: str,
    include_points: bool = Query(True),
    max_points: int = Query(4096, ge=1),
    decimals: int = Query(2, ge=0, le=6),
    include_meta: bool = Query(True),
    filter_invalid: bool = Query(True),
    min_quality: int = Query(1, ge=0),
    min_range_mm: int = Query(1, ge=0),
):
    adapter = _get_adapter_by_id(sensor_id)
    if not adapter:
        raise HTTPException(status_code=404, detail="sensor not found")

    sample = manager.latest(sensor_id)
    if not sample:
        raise HTTPException(status_code=404, detail="sensor or sample not found")
    data = sample.data if isinstance(sample.data, dict) else {}

    # robot transform
    t = adapter.get_transform()
    out: Dict[str, Any] = {"sensor_id": sensor_id, "frame": "robot"}

    if include_meta:
        ts = getattr(sample, "ts", None)
        out["ts"] = ts.isoformat().replace("+00:00", "Z") if ts else None

    # CASE A: modern points list with angle/range (2D lidar)
    if include_points and "points" in data and isinstance(data["points"], list):
        points = data["points"][:max_points]
        if filter_invalid:
            filt = []
            for p in points:
                q = int(p.get("quality", 0) or 0)
                rmm = float(p.get("distance_mm", 0.0) or 0.0)
                if q >= min_quality and rmm >= min_range_mm:
                    filt.append(p)
            points = filt

        xyz = []
        for p in points:
            ang = float(p.get("angle_deg", 0.0))
            rmm = float(p.get("distance_mm", 0.0))
            rad = math.radians(ang)
            m = rmm / 1000.0
            x = m * math.cos(rad)
            y = m * math.sin(rad)
            xr, yr, zr = apply_transform_xyz(x, y, 0.0, t)  # type: ignore
            xyz.append({"x": round(xr, decimals), "y": round(yr, decimals), "z": round(zr, decimals), "quality": int(p.get("quality", 0))})

        out["data"] = {"points_xyz": xyz}
        out["transform"] = getattr(t, "to_dict", lambda: t)()  # type: ignore
        return out

    # CASE B: legacy arrays (angles/ranges/qualities)
    angles = list(data.get("angles", []))
    ranges = list(data.get("ranges", []))
    qualities = list(data.get("qualities", []))

    if include_points and len(angles) and len(ranges) and len(qualities):
        n = min(len(angles), len(ranges), len(qualities))
        angles, ranges, qualities = angles[:n], ranges[:n], qualities[:n]

        if filter_invalid:
            filt_ang, filt_rng, filt_qual = [], [], []
            for a, r, q in zip(angles, ranges, qualities):
                q = int(q or 0); r = float(r or 0.0)
                if q >= min_quality and r >= min_range_mm:
                    filt_ang.append(a); filt_rng.append(r); filt_qual.append(q)
            angles, ranges, qualities = filt_ang, filt_rng, filt_qual

        step = max(1, len(angles) // max_points) if len(angles) > max_points else 1
        angles, ranges, qualities = angles[::step], ranges[::step], qualities[::step]

        xyz = []
        for a, rmm, q in zip(angles, ranges, qualities):
            rad = math.radians(float(a)); m = float(rmm)/1000.0
            x = m * math.cos(rad); y = m * math.sin(rad)
            xr, yr, zr = apply_transform_xyz(x, y, 0.0, t)  # type: ignore
            xyz.append({"x": round(xr, decimals), "y": round(yr, decimals), "z": round(zr, decimals), "quality": int(q)})

        out["data"] = {"points_xyz": xyz}
        out["transform"] = getattr(t, "to_dict", lambda: t)()  # type: ignore
        return out

    # CASE C: already XYZ points (e.g., 3D lidar)
    if include_points and "points_xyz" in data and isinstance(data["points_xyz"], list):
        points3d = data["points_xyz"][:max_points]
        xyz = []
        for p in points3d:
            xr, yr, zr = apply_transform_xyz(float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("z", 0.0)), t)  # type: ignore
            xyz.append({"x": round(xr, decimals), "y": round(yr, decimals), "z": round(zr, decimals)})
        out["data"] = {"points_xyz": xyz}
        out["transform"] = getattr(t, "to_dict", lambda: t)()  # type: ignore
        return out

    raise HTTPException(status_code=204, detail="no points in latest sample")

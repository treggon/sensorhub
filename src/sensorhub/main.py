
# src/sensorhub/main.py
import os
import traceback
from pathlib import Path
from fastapi import FastAPI, HTTPException
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
from sensorhub.api.sensors_latest import router as latest_router
from sensorhub.api.snapshot import router as snapshot_router
from sensorhub.api.livox_snapshot import router as livox_snapshot_router

# Configure logging
configure_logging()

# Force rplidar adapter loggers to DEBUG (ensures raw stdout lines are visible)
import logging
logging.getLogger("sensorhub.adapters.rplidar_s2").setLevel(logging.DEBUG)
logging.getLogger("sensorhub.adapters.rplidar_s2.rplidar_adapter").setLevel(logging.DEBUG)

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
app.add_middleware(GZipMiddleware, minimum_size=1024)

# Routers
app.include_router(health_router)
app.include_router(sensors_router)
app.include_router(video_router)
app.include_router(ws_router)
app.include_router(livox_router)
app.include_router(snapshot_router)
app.include_router(livox_snapshot_router)
app.include_router(latest_router)

# Helper: locate adapter by id
def _get_adapter_by_id(sensor_id: str):
    # Try common manager methods
    for meth in ("get_adapter", "get_sensor", "get"):
        if hasattr(manager, meth):
            try:
                return getattr(manager, meth)(sensor_id)
            except Exception:
                pass
    # Try common registries
    for attr in ("adapters", "_adapters", "sensors", "_registry"):
        if hasattr(manager, attr):
            reg = getattr(manager, attr)
            if isinstance(reg, dict):
                return reg.get(sensor_id)
            if isinstance(reg, (list, tuple)):
                for a in reg:
                    if getattr(a, "sensor_id", None) == sensor_id:
                        return a
    return None

@app.get("/sensors/{sensor_id}/latest_raw")
def get_latest_raw(sensor_id: str):
    adapter = _get_adapter_by_id(sensor_id)
    if not adapter or not hasattr(adapter, "get_latest_frame"):
        raise HTTPException(status_code=404, detail="sensor or sample not found")
    frame = adapter.get_latest_frame()
    if frame is None:
        raise HTTPException(status_code=204, detail="no frame yet")
    return frame

@app.on_event("startup")
async def startup_event():
    cfg_path = os.getenv(
        "SENSORHUB_CONFIG",
        str(Path(__file__).parent / "config" / "config.video.yaml"),
    )
    print(f"[SensorHub] Loading sensors from config: {cfg_path}")

    # Robust error logging when loading the config
    try:
        manager.load_from_config(Path(cfg_path))
    except Exception as e:
        print("[SensorHub] ERROR loading config:")
        print(f"  Path: {cfg_path}")
        print(f"  Exception: {e.__class__.__name__}: {e}")
        print("  Traceback:")
        traceback.print_exc()
        raise

@app.get("/")
def root():
    return {"ok": True, "service": "SensorHub", "version": "0.3.2"}

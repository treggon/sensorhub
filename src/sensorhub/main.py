
import os
from pathlib import Path
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api.routes import router as sensors_router
from .api.ws import router as ws_router
from .core.sensor_manager import manager
from .logging_config import configure_logging
from .api.health import router as health_router
from .api.video import router as video_router
from sensorhub.adapters.livox_mid360.livox_adapter import router as livox_router

configure_logging()

app = FastAPI(title="SensorHub",
              description="Modular API/WebSocket service for robot sensors (with USB camera streaming)",
              version="0.3.1")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(health_router)
app.include_router(sensors_router)
app.include_router(video_router)
app.include_router(ws_router)
app.include_router(livox_router)

@app.on_event("startup")
async def startup_event():
    cfg_path = os.getenv('SENSORHUB_CONFIG', str(Path(__file__).parent / 'config' / 'config.video.yaml'))
    manager.load_from_config(Path(cfg_path))

# Run: uvicorn sensorhub.main:app --host 0.0.0.0 --port 8080

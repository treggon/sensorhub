
# src/sensorhub/api/summary.py
from pathlib import Path
from typing import Any, Dict, List, Optional
import time

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.templating import Jinja2Templates

from ..core.sensor_manager import manager

# PipelineManager may not exist in some builds; guard import
try:
    from ..video.pipeline_manager import PipelineManager
except Exception:
    PipelineManager = None  # type: ignore

router = APIRouter(prefix="", tags=["summary"])

# Templates live at: src/sensorhub/templates/
TEMPLATES_DIR = Path(__file__).resolve().parent.parent / "templates"
templates = Jinja2Templates(directory=str(TEMPLATES_DIR))

pm: Optional[PipelineManager] = PipelineManager() if PipelineManager else None


def normalize_status(payload: Optional[Dict[str, Any]], default: str = "unknown") -> str:
    """Map various shapes to: ok | warning | error | unknown."""
    if not payload:
        return default
    s = str(payload.get("status", "")).lower()
    if s in {"ok", "healthy", "ready"}:
        return "ok"
    if s in {"degraded", "warning"}:
        return "warning"
    if s in {"error", "fail", "down", "notready"}:
        return "error"
    # Boolean forms
    for key in ("healthy", "ready", "video", "running"):
        if key in payload:
            return "ok" if bool(payload[key]) else "error"
    return default


@router.get("/api/summary", response_class=JSONResponse)
async def summary_json() -> JSONResponse:
    """Aggregate system + sensor health into a compact JSON payload."""
    # --- System health (mirror your health endpoints) ---
    sys_health = {"status": "ok"}
    ready_any = any(getattr(a, "latest", None) is not None for a in getattr(manager, "adapters", {}).values())
    sys_ready = {"ready": ready_any}

    cams = [a.sensor_id for a in getattr(manager, "adapters", {}).values() if getattr(a, "kind", None) == "camera"]
    pipelines: List[Dict[str, Any]] = []
    if pm:
        try:
            pipelines = [
                {"id": s.id, "running": s.running, "backend": s.backend, "rtsp_url": s.rtsp_url}
                for s in pm.list()
            ]
        except Exception:
            pipelines = []

    system = {
        "health": normalize_status(sys_health),
        "ready": normalize_status(sys_ready),
        "video": "ok" if any(p.get("running") for p in pipelines) else ("unknown" if not pipelines else "error"),
        "timestamp": int(time.time()),
    }

    # --- Sensors ---
    sensors_info: List[Dict[str, Any]] = manager.list_with_status() or []
    sensors: List[Dict[str, Any]] = []
    for s in sensors_info:
        sid = s.get("id") or s.get("sensor_id")
        st = manager.get_status(sid) or {}
        sensors.append({
            "id": sid,
            "name": s.get("name") or sid,
            "type": s.get("kind") or s.get("type"),
            "health": normalize_status(st),
            "urls": {
                "health": f"/sensors/{sid}/health",
                "latest": f"/sensors/{sid}/latest",
                "latest_raw": f"/sensors/{sid}/latest_raw",
                "history": f"/sensors/{sid}/history",
                "snapshot_png": f"/sensors/{sid}/snapshot.png",
            }
        })

    return JSONResponse(content={
        "system": system,
        "sensors": sensors,
        "video": {"cameras": cams, "pipelines": pipelines},
    })


@router.get("/summary", response_class=HTMLResponse)
async def summary_page(request: Request) -> HTMLResponse:
    """Render the dashboard shell; the page fetches /api/summary every 2s."""
    return templates.TemplateResponse("summary.html", {"request": request})

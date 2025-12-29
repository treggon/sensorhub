
from fastapi import APIRouter
from ..core.sensor_manager import manager
from ..video.pipeline_manager import PipelineManager

router = APIRouter(prefix="", tags=["health"])
pm = PipelineManager()

@router.get("/health")
async def health():
    return {"status": "ok"}

@router.get("/ready")
async def ready():
    ready_any = any(a.latest is not None for a in manager.adapters.values())
    return {"ready": ready_any}

@router.get("/health/video")
async def health_video():
    cams = [a.sensor_id for a in manager.adapters.values() if a.kind == "camera"]
    pipelines = [{"id": s.id, "running": s.running, "backend": s.backend, "rtsp_url": s.rtsp_url} for s in pm.list()]
    return {"cameras": cams, "pipelines": pipelines}

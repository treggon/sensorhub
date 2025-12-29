
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from ..video.pipeline_manager import PipelineManager, PipelineSpec, SourceSpec, RTSPSettings

router = APIRouter(prefix="/video", tags=["video-pipelines"])
pm = PipelineManager()

class SourceModel(BaseModel):
    type: str = Field(pattern="^(device|rtsp)$")
    device: str | None = None
    url: str | None = None

class RTSPModel(BaseModel):
    enable: bool = True
    name: str
    bitrate_kbps: int = 2500
    fps: int = 30
    width: int = 1280
    height: int = 720
    gop: int = 60
    codec: str = Field(pattern="^(h264|vp8|vp9)$")
    low_latency: bool = True

class PipelineModel(BaseModel):
    id: str
    backend: str = Field(pattern="^(ffmpeg|gstreamer)$")
    source: SourceModel
    rtsp: RTSPModel
    target_host: str = "127.0.0.1"
    target_port: int = 8554
    mjpeg: bool = True

@router.post("/pipelines")
def create_pipeline(m: PipelineModel):
    spec = PipelineSpec(
        id=m.id,
        backend=m.backend, 
        source=SourceSpec(type=m.source.type, device=m.source.device, url=m.source.url),
        rtsp=RTSPSettings(**m.rtsp.dict()),
        target_host=m.target_host,
        target_port=m.target_port,
        mjpeg_enable=m.mjpeg,
    )
    st = pm.start(spec)
    return {"status": "started", "pipeline": st.__dict__}

@router.get("/pipelines")
def list_pipelines():
    return [s.__dict__ for s in pm.list()]

class PatchModel(BaseModel):
    backend: str | None = None
    source_type: str | None = None
    device: str | None = None
    url: str | None = None
    name: str | None = None
    bitrate_kbps: int | None = None
    fps: int | None = None
    width: int | None = None
    height: int | None = None
    gop: int | None = None
    codec: str | None = None
    low_latency: bool | None = None
    target_host: str | None = None
    target_port: int | None = None

@router.patch("/pipelines/{pid}")
def patch_pipeline(pid: str, m: PatchModel):
    st = pm.patch(pid, **{k: v for k, v in m.dict().items() if v is not None})
    if not st: raise HTTPException(status_code=404, detail="pipeline not found")
    return {"status": "patched", "pipeline": st.__dict__}

@router.delete("/pipelines/{pid}")
def delete_pipeline(pid: str):
    ok = pm.stop(pid)
    if not ok: raise HTTPException(status_code=404, detail="pipeline not found")
    return {"status": "stopped", "id": pid}

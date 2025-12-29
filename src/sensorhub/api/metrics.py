
import os, time
from typing import Any
from fastapi import APIRouter
from ..video.pipeline_manager import PipelineManager

router = APIRouter(prefix="/metrics", tags=["metrics"])
pm = PipelineManager()  # reuse the manager instance; if you keep a global one, import it instead

def _read_meminfo() -> dict[str, int]:
    out = {}
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                k, v = line.split(":")[0], line.split(":")[1].strip()
                val = int(v.split()[0]) if v.split() else 0
                out[k] = val  # kB
    except Exception:
        pass
    return out

@router.get("/system")
def system_metrics():
    load = os.getloadavg() if hasattr(os, "getloadavg") else (0.0, 0.0, 0.0)
    mem = _read_meminfo()
    return {
        "timestamp": time.time(),
        "loadavg_1_5_15": load,
        "mem_kb": {k: mem.get(k) for k in ("MemTotal", "MemFree", "Buffers", "Cached")}
    }

@router.get("/video")
def video_metrics():
    rows: list[dict[str, Any]] = []
    for s in pm.list():
        rows.append({
            "id": s.id, "backend": s.backend, "pid": s.pid, "running": s.running,
            "codec": s.codec, "fps": s.fps, "bitrate_kbps": s.bitrate_kbps,
            "size": f"{s.width}x{s.height}", "gop": s.gop, "rtsp_url": s.rtsp_url
        })
    return {"pipelines": rows}

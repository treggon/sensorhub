
import asyncio
from typing import List
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse, Response
from ..core.sensor_manager import manager
from ..core.schemas import SensorInfo

router = APIRouter(prefix='/video', tags=['video'])

@router.get('/cameras', response_model=List[SensorInfo])
async def list_cameras():
    return [SensorInfo(id=a.sensor_id, kind=a.kind) for a in manager.adapters.values() if a.kind == 'camera']

@router.get('/{camera_id}/snapshot.jpg')
async def snapshot(camera_id: str):
    adapter = manager.adapters.get(camera_id)
    if not adapter or adapter.kind != 'camera':
        raise HTTPException(status_code=404, detail='camera not found')
    # The adapter exposes latest JPEG via attribute for streaming endpoints
    jpeg = getattr(adapter, 'latest_jpeg', None)
    if not jpeg:
        raise HTTPException(status_code=404, detail='no frame yet')
    return Response(content=jpeg, media_type='image/jpeg')

@router.get('/{camera_id}/mjpeg')
async def mjpeg(camera_id: str):
    adapter = manager.adapters.get(camera_id)
    if not adapter or adapter.kind != 'camera':
        raise HTTPException(status_code=404, detail='camera not found')

    async def frame_gen():
        boundary = 'frame'
        while True:
            jpeg = getattr(adapter, 'latest_jpeg', None)
            if jpeg:
                yield (b"--" + boundary.encode() + b"
"
                       b"Content-Type: image/jpeg
"
                       b"Content-Length: " + str(len(jpeg)).encode() + b"

" + jpeg + b"
")
            await asyncio.sleep(getattr(adapter, 'frame_interval', 0.033))

    return StreamingResponse(frame_gen(), media_type='multipart/x-mixed-replace; boundary=frame')

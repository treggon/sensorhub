
from fastapi import APIRouter, HTTPException
from ..core.sensor_manager import manager
from ..core.schemas import SensorInfo, Sample, HistoryResponse

router = APIRouter(prefix='/sensors', tags=['sensors'])

@router.get('', response_model=list[SensorInfo])
async def list_sensors():
    return manager.list()

@router.get('/{sensor_id}/latest', response_model=Sample)
async def latest(sensor_id: str):
    s = manager.latest(sensor_id)
    if not s:
        raise HTTPException(status_code=404, detail='sensor or sample not found')
    return s

@router.get('/{sensor_id}/history', response_model=HistoryResponse)
async def history(sensor_id: str, limit: int = 100):
    return HistoryResponse(sensor_id=sensor_id, samples=manager.history(sensor_id, limit=limit))

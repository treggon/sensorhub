
from fastapi import APIRouter
from ..core.sensor_manager import manager

router = APIRouter(prefix='', tags=['health'])

@router.get('/health')
async def health():
    return {'status': 'ok'}

@router.get('/ready')
async def ready():
    return {'ready': any(a.latest is not None for a in manager.adapters.values())}

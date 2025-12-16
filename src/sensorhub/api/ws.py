
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from ..core.sensor_manager import manager

router = APIRouter()

@router.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        subscriptions: set[str] = set()
        while True:
            msg = await ws.receive_json()
            action = msg.get('action')
            if action == 'subscribe':
                sid = msg.get('sensor_id')
                if sid in manager.adapters:
                    subscriptions.add(sid)
                    await ws.send_json({'type': 'subscribed', 'sensor_id': sid})
                else:
                    await ws.send_json({'type': 'error', 'error': f'unknown sensor {sid}'})
            elif action == 'poll':
                out = {}
                for sid in list(subscriptions):
                    s = manager.latest(sid)
                    if s:
                        out[sid] = s.dict()
                await ws.send_json({'type': 'poll-result', 'data': out})
            else:
                await ws.send_json({'type': 'error', 'error': 'unknown action'})
    except WebSocketDisconnect:
        return

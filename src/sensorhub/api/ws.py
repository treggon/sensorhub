
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.encoders import jsonable_encoder
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
                if not isinstance(sid, str):
                    await ws.send_json({'type': 'error', 'error': 'sensor_id must be a string'})
                    continue

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
                        # If s is a Pydantic model, .dict() is fine, but it may contain datetimes.
                        # jsonable_encoder below will convert nested datetimes into ISO strings.
                        out[sid] = s.dict()

                # Convert any datetimes (and other non-JSON-native types) into JSON-serializable values.
                payload = {'type': 'poll-result', 'data': jsonable_encoder(out)}
                await ws.send_json(payload)

            else:
                await ws.send_json({'type': 'error', 'error': 'unknown action'})
    except WebSocketDisconnect:
        return

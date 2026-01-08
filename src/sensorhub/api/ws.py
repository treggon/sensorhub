
# src/sensorhub/api/ws.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.encoders import jsonable_encoder
import asyncio
import time

from ..core.sensor_manager import manager  # your existing manager
router = APIRouter()

@router.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        # Subscriptions per connection
        poll_subs: set[str] = set()
        push_subs: dict[str, dict] = {}   # sid -> {"push_hz": float, "last_ts": str|None, "task": asyncio.Task|None}

        async def start_pusher_for(sid: str, push_hz: float):
            """Background loop: send 'frame' on changes at up to push_hz."""
            interval = 1.0 / max(1.0, push_hz)
            push_subs[sid]["last_ts"] = None
            # simple loop that checks manager.latest and sends when ts changes
            while True:
                await asyncio.sleep(interval)
                latest = manager.latest(sid)  # returns a Pydantic model or None
                if not latest:
                    continue
                sample = latest.dict()  # model -> dict
                ts = sample.get("ts") or sample.get("timestamp")  # support both keys
                if ts != push_subs[sid]["last_ts"]:
                    push_subs[sid]["last_ts"] = ts
                    payload = {"type": "frame", "sensor_id": sid, "data": jsonable_encoder(sample)}
                    try:
                        await ws.send_json(payload)
                    except RuntimeError:
                        # socket likely closed
                        break

        while True:
            msg = await ws.receive_json()
            action = msg.get("action")

            if action == "subscribe":
                sid = msg.get("sensor_id")
                if not isinstance(sid, str):
                    await ws.send_json({"type": "error", "error": "sensor_id must be a string"})
                    continue
                if sid not in manager.adapters:
                    await ws.send_json({"type": "error", "error": f"unknown sensor {sid}"})
                    continue

                mode = (msg.get("mode") or "poll").lower().strip()
                if mode == "push":
                    push_hz = float(msg.get("push_hz") or 10.0)
                    # create pusher task for this sid if not running
                    if sid not in push_subs or push_subs[sid].get("task") is None or push_subs[sid]["task"].done():
                        push_subs[sid] = {"push_hz": push_hz, "last_ts": None, "task": asyncio.create_task(start_pusher_for(sid, push_hz))}
                    await ws.send_json({"type": "subscribed", "sensor_id": sid, "mode": "push", "push_hz": push_hz})
                else:
                    # default poll mode
                    poll_subs.add(sid)
                    await ws.send_json({"type": "subscribed", "sensor_id": sid, "mode": "poll"})

            elif action == "poll":
                out = {}
                for sid in list(poll_subs):
                    latest = manager.latest(sid)
                    if latest:
                        out[sid] = jsonable_encoder(latest.dict())
                await ws.send_json({"type": "poll-result", "data": out})

            elif action == "unsubscribe":
                sid = msg.get("sensor_id")
                if isinstance(sid, str):
                    poll_subs.discard(sid)
                    # stop push task if exists
                    if sid in push_subs and push_subs[sid].get("task"):
                        t = push_subs[sid]["task"]
                        try:
                            t.cancel()
                        except Exception:
                            pass
                        push_subs[sid]["task"] = None
                    await ws.send_json({"type": "unsubscribed", "sensor_id": sid})
                else:
                    await ws.send_json({"type": "error", "error": "sensor_id must be a string"})

            else:
                await ws.send_json({"type": "error", "error": "unknown action"})

    except WebSocketDisconnect:
        # clean up tasks
        for sid, meta in list(push_subs.items()):
            t = meta.get("task")
            if t:
                try:
                    t.cancel()
                except Exception:
                    pass
        return

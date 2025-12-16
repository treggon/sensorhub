
import asyncio
import websockets
import json

async def main():
    async with websockets.connect('ws://localhost:8080/ws') as ws:
        await ws.send(json.dumps({'action': 'subscribe', 'sensor_id': 'sim1'}))
        while True:
            await ws.send(json.dumps({'action': 'poll'}))
            msg = await ws.recv()
            print(msg)
            await asyncio.sleep(0.1)

if __name__ == '__main__':
    asyncio.run(main())

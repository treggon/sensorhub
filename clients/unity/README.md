
# Unity RPLidar Client & Visualizer

A plug‑and‑play Unity client that connects to your FastAPI WebSocket endpoint and renders live RPLidar scans. It includes:

- **Robust WebSocket client** (auto‑reconnect, heartbeat, re‑subscribe, flexible JSON parsing)
- **Four visualization modes**: Raw Points, Line Renderer, Mesh Point Cloud (CPU quads), and GPU Shader
- **Composite visualizer** to **layer multiple modes at once**
- Optional **runtime mode switcher** (keys `1–4`) for quick comparisons

> This README assumes your FastAPI app exposes a WebSocket at `ws://<host>:<port>/ws` and publishes `poll-result` frames that include the latest `rplidar_s2` scan. See the **Server** section below for a known‑good `ws.py`.

---

## 0) Requirements

- **Unity** 2020.3+ (Built‑in or URP; HDRP needs a small shader tweak)
- **Newtonsoft.Json** package: `com.unity.nuget.newtonsoft-json` (Install via *Window → Package Manager*)
- Your FastAPI app running (e.g., `uvicorn sensorhub.main:app --host 0.0.0.0 --port 8082`)

---

## 1) Project Files (scripts & shader)

> Place these under `Assets/Scripts` (and `Assets/Shaders` for the shader). You already have the scripts from our conversation—summarized here for clarity.

### 1.1 SensorHubWebSocketClient.cs (features)

- Connects to `ws://<host>:<port>/ws`
- **Auto‑reconnect** with exponential backoff + jitter
- **Heartbeat** (`ping`/`pong`) + **stale timeout** (forces reconnect)
- **Auto re‑subscribe** to all sensors after reconnect
- **Flexible payload parsing** (`angles`+`distances`, `points[{angle,distance}]`, nested `data`)
- **Auto‑detect units** (radians→degrees, millimeters→meters via distance median)
- Thread‑safe **queue** to pass data to Unity main thread
- Sends data to either a single **RPLidarVisualizer** or a **composite** (RPLidarVisualizerComposite)

> Inspector fields: `serverUrl`, `sensorIds`, `lidarSensorId`, `lidarVisualizer` or `lidarComposite`, `pollIntervalMs`, `pingIntervalMs`, `connectionTimeoutMs`, backoff settings.

### 1.2 RPLidarVisualizer.cs (modes)

Renders scans in Unity’s **XZ plane** (top‑down). Modes:

- **RawPoints**: pooled prefab dots (simple, great for debugging)
- **LineRenderer**: continuous ring/contour (very fast)
- **MeshPoints**: CPU‑built quads; good for a few thousand points
- **GPUShader**: `ComputeBuffer` + `Graphics.DrawProcedural` (best for many points)

Key fields:
- `distanceScale` (0.001 for mm, 1.0 for meters)
- `minDistance`, `maxDistance` (0 disables max)
- `downsample`
- Mode‑specific assets: `pointPrefab`, `lineRenderer`, `meshMaterial`, `gpuMaterial`, sizes

### 1.3 RPLidarVisualizerComposite.cs (optional)

A tiny forwarder that lets you **layer multiple visualizers**. Add several `RPLidarVisualizer` components on different GameObjects (GPU, Line, RawPoints), then reference them in the composite. Your WebSocket client sends scans to the composite, which forwards to each visualizer.

### 1.4 ModeSwitcher.cs (optional)

Press `1–4` to swap modes on a single `RPLidarVisualizer`, or toggle layers on a composite.

### 1.5 Shaders

Two options—pick **one**:

- **Built‑in/URP Unlit** point‑sprite shader: `Assets/Shaders/RPLidarPoints.shader`
- **URP‑specific** variant: `Assets/Shaders/RPLidarPoints_URP.shader`

> Both are provided in the conversation. If your project is HDRP, ask for the HDRP pass.

---

## 2) Server (FastAPI) – WebSocket endpoint

Use this `ws.py` (already provided) so `datetime` and nested types serialize cleanly via `jsonable_encoder`:

```python
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.encoders import jsonable_encoder
from typing import Set, Dict, Any
import json, logging
from ..core.sensor_manager import manager

logger = logging.getLogger("uvicorn")
router = APIRouter()

def encode_latest(value: Any) -> Any:
    for attr in ("model_dump", "dict"):
        if hasattr(value, attr):
            fn = getattr(value, attr)
            if callable(fn):
                try:
                    value = fn()
                    break
                except Exception:
                    pass
    return jsonable_encoder(value)

@router.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    logger.info("[WS] client connected")
    subs: Set[str] = set()
    try:
        while True:
            msg_text = await ws.receive_text()
            try:
                msg = json.loads(msg_text)
            except json.JSONDecodeError:
                await ws.send_json({"type":"error","error":"invalid JSON"}); continue
            action = msg.get("action")
            if action == "subscribe":
                sid = msg.get("sensor_id")
                if sid in manager.adapters:
                    subs.add(sid)
                    await ws.send_json({"type":"subscribed","sensor_id":sid})
                    logger.info(f"[WS] subscribed -> {sid} (total {len(subs)})")
                else:
                    await ws.send_json({"type":"error","error":f"unknown sensor '{sid}'"})
            elif action == "poll":
                out: Dict[str, Any] = {}
                for sid in list(subs):
                    latest = manager.latest(sid)
                    if latest is not None:
                        out[sid] = encode_latest(latest)
                await ws.send_json({"type":"poll-result","data": jsonable_encoder(out)})
                logger.info(f"[WS] poll-result -> {len(out)} items: {list(out.keys())}")
            elif action == "ping":
                await ws.send_json({"type":"pong"})
            else:
                await ws.send_json({"type":"error","error":"unknown action"})
    except WebSocketDisconnect:
        logger.info("[WS] client disconnected")
        return
```

Run:
```bash
uvicorn sensorhub.main:app --reload --host 0.0.0.0 --port 8082 --app-dir src
```

---

## 3) Unity Setup – step‑by‑step

### 3.1 Install Newtonsoft.Json
- **Window → Package Manager → Unity Registry → Newtonsoft.Json → Install**

### 3.2 Add scripts
- Copy the four scripts into `Assets/Scripts/`.
- Add **SensorHubWebSocketClient** to any GameObject (e.g., `NetworkManager`).
- In the Inspector:
  - `serverUrl = ws://<host>:8082/ws`
  - `sensorIds = ["sim1", "gps1", "rplidar_s2"]` (ensure they exist server‑side)
  - Assign either **`lidarVisualizer`** (single) or **`lidarComposite`** (multi‑layer)
  - Set `pollIntervalMs = 100`, `pingIntervalMs = 5000`, `connectionTimeoutMs = 15000`

### 3.3 Create the visualizers

**A) RawPoints**
1. Create `PointPrefab` (Sphere or Quad), scale ≈ `0.02`, optional Unlit material.
2. In `RPLidarVisualizer` (on `LidarDisplay`): set **Mode = RawPoints**, assign `pointPrefab`, set `maxRawPoints` ≥ 4000.

**B) LineRenderer**
1. Add a LineRenderer to `LidarDisplay`.
2. Set **Use World Space = OFF**, **Loop = ON**, **Width = 0.02**.
3. Create `LineMat` (Unlit/Color) and assign it to the LineRenderer.
4. In `RPLidarVisualizer`: **Mode = LineRenderer**, assign LineRenderer.

**C) Mesh Point Cloud (CPU)**
1. Create `PointCloudMat` (Unlit/Color).
2. In `RPLidarVisualizer`: **Mode = MeshPoints**, assign `meshMaterial = PointCloudMat`, set `pointQuadSize = 0.02`.

**D) GPU Shader**
1. Save `Assets/Shaders/RPLidarPoints.shader` (or `_URP.shader`).
2. Create `LidarGPUMat` using that shader.
3. In `RPLidarVisualizer`: **Mode = GPUShader**, assign `gpuMaterial = LidarGPUMat`, set `gpuPointSize = 6–10`.

> For **URP** projects: prefer the `_URP` variant of the shader if the built‑in include causes warnings.

### 3.4 Composite layering (optional)
- Create siblings: `Lidar_GPU`, `Lidar_Line`, `Lidar_Picks`, each with a `RPLidarVisualizer` in different modes.
- Create an empty `LidarComposite` with `RPLidarVisualizerComposite` and add the three visualizers to the list.
- In the WebSocket client, assign `lidarComposite`.

### 3.5 Camera & filters
- Place a **top‑down camera** above the origin (e.g., pos `(0, 3, 0)`, rot `(90, 0, 0)`).
- Start with filters disabled: `minDistance = 0`, `maxDistance = 0`, `downsample = 1`.
- The client auto‑sets `distanceScale` (0.001 for mm, 1.0 for meters) based on median; you can override in the visualizer.

---

## 4) Runtime helpers

### ModeSwitcher.cs
```csharp
using UnityEngine;
public class ModeSwitcher : MonoBehaviour
{
    public RPLidarVisualizer viz;
    void Update()
    {
        if (!viz) return;
        if (Input.GetKeyDown(KeyCode.Alpha1)) viz.mode = RPLidarVisualizer.VisualizationMode.RawPoints;
        if (Input.GetKeyDown(KeyCode.Alpha2)) viz.mode = RPLidarVisualizer.VisualizationMode.LineRenderer;
        if (Input.GetKeyDown(KeyCode.Alpha3)) viz.mode = RPLidarVisualizer.VisualizationMode.MeshPoints;
        if (Input.GetKeyDown(KeyCode.Alpha4)) viz.mode = RPLidarVisualizer.VisualizationMode.GPUShader;
    }
}
```

### Composite toggles
```csharp
using UnityEngine;
public class VisualizerComboController : MonoBehaviour
{
    public RPLidarVisualizer gpuViz;
    public RPLidarVisualizer lineViz;
    public RPLidarVisualizer picksViz;
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) picksViz.enabled = !picksViz.enabled;
        if (Input.GetKeyDown(KeyCode.Alpha2)) lineViz.enabled  = !lineViz.enabled;
        if (Input.GetKeyDown(KeyCode.Alpha3)) gpuViz.enabled   = !gpuViz.enabled;
    }
}
```

---

## 5) Troubleshooting

- **Subscribed but no points**: Confirm server logs show `poll-result -> N items`. If `0`, your manager isn’t producing samples; fix upstream. If `N>0`, add raw RX logging in the client and verify keys.
- **Units look wrong**: Set `distanceScale` explicitly (`1.0` meters or `0.001` mm) in the visualizer.
- **Shader parse error**: Use the provided `RPLidarPoints.shader` verbatim; ensure file extension `.shader`, valid Shader name, and blocks closed (`ENDCG`, `ENDHLSL`).
- **LineRenderer invisible**: Assign the LineRenderer in visualizer, **Use World Space = OFF**, **Loop = ON**, camera top‑down.
- **GPU invisible**: Ensure `gpuMaterial` uses `Hidden/RPLidarPoints` (or `_URP`), increase `gpuPointSize`, verify camera near/far planes.
- **Performance**: For >10k points, prefer **GPU**. Downsample RawPoints; MeshPoints fine for a few thousand. LineRenderer is very light.

---

## 6) Configuration Cheat Sheet

- **WebSocket**: `serverUrl = ws://<host>:<port>/ws`, `pollIntervalMs ≈ 100`, `pingIntervalMs ≈ 5000`, `connectionTimeoutMs ≈ 15000`
- **Visualizer**: `minDistance = 0`, `maxDistance = 0 (disabled)`, `downsample = 1`
- **Units**: meters → `distanceScale = 1.0`; millimeters → `0.001`
- **Line**: width `0.02`, loop ON, world space OFF
- **Mesh**: quad size `0.01–0.03`; Unlit material
- **GPU**: point size `6–10`; Transparent, `ZWrite Off`

---

## 7) Credits & License

This setup is intended for rapid prototyping and internal tooling. Use freely in your project; attribution appreciated.

If you want a **.unitypackage** that auto‑drops the scripts, shader, materials, and an example scene, let me know—I’ll generate it.

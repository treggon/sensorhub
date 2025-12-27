
"""
Livox Mid-360 Adapter for SensorHub

- Matches SensorManager signature: __init__(sensor_id, kind, **params)
- Loads & validates JSON config (src/sensorhub/config/mid360_config.json)
- Spawns the C++ livox_bridge (linked to Livox SDK2) and ingests NDJSON frames
- Exposes REST + WebSocket endpoints via FastAPI router
"""

import asyncio
import json
import os
import signal
import socket
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException

# Pull in your AbstractSensorAdapter base
from sensorhub.core.sensor_base import AbstractSensorAdapter


# --------------------------
# Config & validation pieces
# --------------------------

def _is_ipv4(s: str) -> bool:
    """Return True if 's' is a valid IPv4 address."""
    try:
        socket.inet_aton(s)
        return True
    except OSError:
        return False


class ConfigError(Exception):
    """Raised for invalid Livox JSON config."""
    pass


# Resolve schema & default config from repo paths
SCHEMA_PATH = Path(__file__).resolve().parents[2] / "config" / "mid360_schema.json"
CONFIG_PATH_DEFAULT = Path(__file__).resolve().parents[2] / "config" / "mid360_config.json"

# Load schema once (human-readable schema; validation below is manual & strict)
SCHEMA = json.loads(SCHEMA_PATH.read_text()) if SCHEMA_PATH.exists() else {}


def validate_config(cfg: Dict[str, Any]) -> None:
    """
    Strict validation for livox multi-device config.
    Ensures required keys, IPv4 formats, port ranges, and disallows unknown keys.
    """
    errors: List[str] = []

    # Basic shape
    if "lidars" not in cfg or not isinstance(cfg["lidars"], list) or len(cfg["lidars"]) == 0:
        errors.append("'lidars' must be a non-empty array")
    else:
        for i, l in enumerate(cfg["lidars"]):
            if not isinstance(l, dict):
                errors.append(f"lidars[{i}] must be an object")
                continue

            # Required keys per device
            required = ["id", "lidar_ip", "host_ip", "cmd_data_port", "point_data_port", "imu_data_port"]
            for k in required:
                if k not in l:
                    errors.append(f"lidars[{i}].{k} is required")

            # id
            if "id" in l and (not isinstance(l["id"], str) or not l["id"]):
                errors.append(f"lidars[{i}].id must be a non-empty string")

            # IPv4 keys
            for ipk in ["lidar_ip", "host_ip"]:
                if ipk in l:
                    if not isinstance(l[ipk], str) or not _is_ipv4(l[ipk]):
                        errors.append(f"lidars[{i}].{ipk} must be a valid IPv4 string")

            # Port ranges
            for pk in ["cmd_data_port", "point_data_port", "imu_data_port", "ndjson_udp_port"]:
                if pk in l:
                    v = l[pk]
                    if not isinstance(v, int) or not (1 <= v <= 65535):
                        errors.append(f"lidars[{i}].{pk} must be integer in [1,65535]")

            # Unknown keys
            allowed = {
                "id", "lidar_ip", "host_ip",
                "cmd_data_port", "point_data_port", "imu_data_port",
                "ndjson_udp_port"
            }
            extra = set(l.keys()) - allowed
            if extra:
                errors.append(f"lidars[{i}] contains unknown keys: {sorted(extra)}")

    # Bridge block (optional)
    if "bridge" in cfg:
        b = cfg["bridge"]
        if not isinstance(b, dict):
            errors.append("'bridge' must be an object")
        else:
            if "stdout" in b and not isinstance(b["stdout"], bool):
                errors.append("bridge.stdout must be boolean")
            if "ndjson_udp_port" in b:
                v = b["ndjson_udp_port"]
                if not isinstance(v, int) or not (1 <= v <= 65535):
                    errors.append("bridge.ndjson_udp_port must be integer in [1,65535]")
            extra = set(b.keys()) - {"stdout", "ndjson_udp_port"}
            if extra:
                errors.append(f"bridge contains unknown keys: {sorted(extra)}")

    # Top-level unknown keys
    if set(cfg.keys()) - {"lidars", "bridge"}:
        errors.append("top-level contains unknown keys")

    if errors:
        raise ConfigError("; ".join(errors))


# --------------------------
# Adapter configuration type
# --------------------------

@dataclass
class LivoxAdapterConfig:
    """
    Config structure that the adapter uses internally.
    Values can be overridden via SensorManager params or environment.
    """
    # Network / bridge knobs (defaults align with your current setup)
    lidar_ip: str = os.getenv("LIVOX_LIDAR_IP", "10.0.0.10")
    host_ip: str = os.getenv("LIVOX_HOST_IP", "0.0.0.0")
    cmd_port: int = 56000
    point_port: int = 56301   # match your Viewer setting
    imu_port: int = 58000

    # Bridge binary & ingest port
    bridge_exe: str = os.getenv(
        "LIVOX_BRIDGE_EXE",
        str(Path(__file__).resolve().parent / "bridge" / "build" / "livox_bridge")
    )
    udp_listen_port: int = int(os.getenv("LIVOX_UDP_PORT", "18080"))

    # Transport choice for bridge → adapter
    use_udp: bool = True

    # Buffer sizes
    max_frames: int = 60
    max_stats: int = 300

    # Where the Livox multi-device JSON lives
    config_path: Path = Path(os.getenv("MID360_CONFIG_PATH", str(CONFIG_PATH_DEFAULT)))


# --------------------------
# Livox adapter class
# --------------------------

class LivoxMid360Adapter(AbstractSensorAdapter):
    """
    SensorHub adapter that spawns the Livox C++ bridge and ingests NDJSON frames.

    - Works with SensorManager: __init__(sensor_id, kind, **params)
    - Uses a singleton hook (INSTANCE) so the FastAPI router reuses the same object
    """
    INSTANCE: Optional["LivoxMid360Adapter"] = None  # router will reuse this instance

    def __init__(self, sensor_id: str, kind: str, **params):
        # If your AbstractSensorAdapter requires additional fields, pass them here
        super().__init__(sensor_id=sensor_id, kind=kind)

        # Build cfg from params (overrides environment defaults)
        cfg = LivoxAdapterConfig()
        cfg.host_ip         = params.get("host_ip", cfg.host_ip)
        cfg.lidar_ip        = params.get("lidar_ip", cfg.lidar_ip)
        cfg.cmd_port        = int(params.get("cmd_port", cfg.cmd_port))
        cfg.point_port      = int(params.get("point_port", cfg.point_port))
        cfg.imu_port        = int(params.get("imu_port", cfg.imu_port))
        cfg.bridge_exe      = params.get("bridge_exe", cfg.bridge_exe)
        cfg.udp_listen_port = int(params.get("udp_listen_port", cfg.udp_listen_port))
        cfg.use_udp         = bool(params.get("use_udp", cfg.use_udp))
        cfg.config_path     = Path(params.get("config_path", cfg.config_path))

        self.cfg = cfg

        # Internal state
        self._bridge_proc: Optional[subprocess.Popen] = None
        self._frames: Deque[Dict[str, Any]] = deque(maxlen=cfg.max_frames)
        self._stats: Deque[Dict[str, Any]] = deque(maxlen=cfg.max_stats)
        self._clients: List[WebSocket] = []
        self._udp_task: Optional[asyncio.Task] = None
        self._stdout_task: Optional[asyncio.Task] = None
        self._stop_event: asyncio.Event = asyncio.Event()
        self._config: Dict[str, Any] = {}

        # Load & validate JSON now; fail FastAPI startup early if misconfigured
        self._load_and_validate()

        # Publish singleton for the router to reuse
        LivoxMid360Adapter.INSTANCE = self

    # ----- SensorHub lifecycle -----
    def start(self) -> None:
        """Start the C++ bridge process and begin ingesting NDJSON frames."""
        if self._bridge_proc is not None:
            return

        args = [self.cfg.bridge_exe]
        env = os.environ.copy()
        env["MID360_CONFIG_PATH"] = str(self.cfg.config_path)

        if not self.cfg.use_udp:
            args += ["--stdout"]

        try:
            self._bridge_proc = subprocess.Popen(
                args,
                stdout=subprocess.PIPE if not self.cfg.use_udp else None,
                stderr=subprocess.STDOUT if not self.cfg.use_udp else None,
                text=True,
                bufsize=1,
                env=env
            )
        except FileNotFoundError:
            raise RuntimeError(
                f"livox_bridge not found at {self.cfg.bridge_exe}. "
                "Build it under adapters/livox_mid360/bridge/."
            )

        loop = asyncio.get_event_loop()
        if self.cfg.use_udp:
            self._udp_task = loop.create_task(self._run_udp_listener())
        else:
            assert self._bridge_proc is not None and self._bridge_proc.stdout is not None
            self._stdout_task = loop.create_task(self._run_stdout_reader(self._bridge_proc.stdout))

    def stop(self) -> None:
        """Stop ingestion and terminate the bridge process."""
        self._stop_event.set()
        try:
            if self._udp_task:
                self._udp_task.cancel()
            if self._stdout_task:
                self._stdout_task.cancel()
        finally:
            if self._bridge_proc:
                if self._bridge_proc.poll() is None:
                    self._bridge_proc.send_signal(signal.SIGINT)
                    try:
                        self._bridge_proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self._bridge_proc.kill()
            self._bridge_proc = None

    def run(self) -> None:
        """
        Blocking loop expected by AbstractSensorAdapter.
        SensorManager typically calls this in a worker thread.
        """
        self.start()
        try:
            # Lightweight loop; networking work is done by asyncio tasks
            while not self._stop_event.is_set():
                time.sleep(0.1)
        finally:
            self.stop()

    # ----- Ingestion loops -----
    async def _run_udp_listener(self) -> None:
        """Listen for NDJSON frames on localhost:udp_listen_port."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", self.cfg.udp_listen_port))
        sock.setblocking(False)
        loop = asyncio.get_event_loop()
        try:
            while not self._stop_event.is_set():
                try:
                    data, _ = await loop.run_in_executor(None, sock.recvfrom, 65535)
                except Exception:
                    await asyncio.sleep(0.001)
                    continue
                self._process_line(data.decode("utf-8", errors="ignore"))
        finally:
            sock.close()

    async def _run_stdout_reader(self, pipe) -> None:
        """Read NDJSON frames from the bridge stdout."""
        while not self._stop_event.is_set():
            line = await asyncio.get_event_loop().run_in_executor(None, pipe.readline)
            if not line:
                await asyncio.sleep(0.001)
                continue
            self._process_line(line)

    # ----- Frame processing -----
    def _process_line(self, line: str) -> None:
        """Parse one NDJSON line and route to buffers + WebSocket clients."""
        line = line.strip()
        if not line:
            return
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            self._stats.append({"type": "log", "ts": time.time(), "msg": line})
            return

        if msg.get("type") == "frame":
            self._frames.append(msg)
            asyncio.create_task(self._broadcast(msg))
        else:
            self._stats.append(msg)

    async def _broadcast(self, payload: Dict[str, Any]) -> None:
        """Push frame payload to all connected WebSocket clients."""
        if not self._clients:
            return
        data = json.dumps(payload)
        dead: List[WebSocket] = []
        for ws in list(self._clients):
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            try:
                self._clients.remove(ws)
            except ValueError:
                pass

    # ----- Adapter API helpers -----
    def latest_frame(self) -> Optional[Dict[str, Any]]:
        """Return the most recent frame or None."""
        return self._frames[-1] if self._frames else None

    def recent_frames(self, count: int = 10) -> List[Dict[str, Any]]:
        """Return the last N frames (light-weight sample points per frame)."""
        if count <= 0:
            return []
        return list(self._frames)[-count:]

    def info(self) -> Dict[str, Any]:
        """Return adapter/bridge status."""
        return {
            "sensor_id": getattr(self, "sensor_id", None),  # provided by AbstractSensorAdapter
            "config_path": str(self.cfg.config_path),
            "udp_listen_port": self.cfg.udp_listen_port,
            "frames_buffered": len(self._frames),
            "stats_buffered": len(self._stats),
            "bridge_running": self._bridge_proc is not None and self._bridge_proc.poll() is None,
        }

    # ----- JSON load/validate -----
    def _load_and_validate(self) -> None:
        """Load and validate the multi-device JSON."""
        if not self.cfg.config_path.exists():
            raise ConfigError(f"Config file not found: {self.cfg.config_path}")
        data = json.loads(self.cfg.config_path.read_text())
        validate_config(data)
        self._config = data


# --------------------------
# FastAPI router (uses INSTANCE)
# --------------------------

router = APIRouter(prefix="/lidar", tags=["lidar"])


def _adapter_or_raise() -> LivoxMid360Adapter:
    if LivoxMid360Adapter.INSTANCE is None:
        raise HTTPException(status_code=503, detail="Livox adapter is not initialized.")
    return LivoxMid360Adapter.INSTANCE


@router.get("/info")
async def get_info() -> Dict[str, Any]:
    return _adapter_or_raise().info()


@router.get("/config")
async def get_config() -> Dict[str, Any]:
    return _adapter_or_raise()._config


@router.post("/start")
async def start_bridge() -> Dict[str, Any]:
    adapter = _adapter_or_raise()
    adapter.start()
    return {"ok": True, "bridge_running": True}


@router.post("/stop")
async def stop_bridge() -> Dict[str, Any]:
    adapter = _adapter_or_raise()
    adapter.stop()
    return {"ok": True}


@router.get("/points/latest")
async def get_latest_frame() -> Dict[str, Any]:
    frame = _adapter_or_raise().latest_frame()
    if frame is None:
        raise HTTPException(status_code=404, detail="No frame yet")
    return frame


@router.get("/points/recent")
async def get_recent_frames(count: int = 10) -> List[Dict[str, Any]]:
    return _adapter_or_raise().recent_frames(count)


@router.websocket("/ws")
async def lidar_ws(ws: WebSocket) -> None:
    adapter = _adapter_or_raise()
    await ws.accept()
    adapter._clients.append(ws)
    try:
        while True:
            try:
                _ = await ws.receive_text()  # keepalive; clients can send pings or small messages
            except WebSocketDisconnect:
                break
            except Exception:
                await asyncio.sleep(0.05)
    finally:
        try:
            adapter._clients.remove(ws)
        except ValueError:
            pass

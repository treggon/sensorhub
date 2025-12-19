
import asyncio
import json
import os
import signal
import socket
import subprocess
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException

# --------------------------
# Config validation helpers
# --------------------------

def _is_ipv4(s: str) -> bool:
    try:
        socket.inet_aton(s)
        return True
    except OSError:
        return False

class ConfigError(Exception):
    pass

SCHEMA = json.loads(Path(__file__).resolve().parents[2].joinpath('config', 'mid360_schema.json').read_text())
CONFIG_PATH_DEFAULT = Path(__file__).resolve().parents[2].joinpath('config', 'mid360_config.json')


def validate_config(cfg: Dict[str, Any]) -> None:
    errors: List[str] = []
    # Basic shape
    if 'lidars' not in cfg or not isinstance(cfg['lidars'], list) or len(cfg['lidars']) == 0:
        errors.append("'lidars' must be a non-empty array")
    else:
        for i, l in enumerate(cfg['lidars']):
            if not isinstance(l, dict):
                errors.append(f"lidars[{i}] must be an object")
                continue
            for k in ['id','lidar_ip','host_ip','cmd_data_port','point_data_port','imu_data_port']:
                if k not in l:
                    errors.append(f"lidars[{i}].{k} is required")
            if 'id' in l and (not isinstance(l['id'], str) or not l['id']):
                errors.append(f"lidars[{i}].id must be a non-empty string")
            for ipk in ['lidar_ip','host_ip']:
                if ipk in l:
                    if not isinstance(l[ipk], str) or not _is_ipv4(l[ipk]):
                        errors.append(f"lidars[{i}].{ipk} must be a valid IPv4 string")
            for pk in ['cmd_data_port','point_data_port','imu_data_port','ndjson_udp_port']:
                if pk in l:
                    v = l[pk]
                    if not isinstance(v, int) or not (1 <= v <= 65535):
                        errors.append(f"lidars[{i}].{pk} must be integer in [1,65535]")
            # additional properties check (simple)
            allowed = {'id','lidar_ip','host_ip','cmd_data_port','point_data_port','imu_data_port','ndjson_udp_port'}
            extra = set(l.keys()) - allowed
            if extra:
                errors.append(f"lidars[{i}] contains unknown keys: {sorted(extra)}")
    if 'bridge' in cfg:
        b = cfg['bridge']
        if not isinstance(b, dict):
            errors.append("'bridge' must be an object")
        else:
            if 'stdout' in b and not isinstance(b['stdout'], bool):
                errors.append("bridge.stdout must be boolean")
            if 'ndjson_udp_port' in b:
                v = b['ndjson_udp_port']
                if not isinstance(v, int) or not (1 <= v <= 65535):
                    errors.append("bridge.ndjson_udp_port must be integer in [1,65535]")
            extra = set(b.keys()) - {'stdout','ndjson_udp_port'}
            if extra:
                errors.append(f"bridge contains unknown keys: {sorted(extra)}")
    if set(cfg.keys()) - {'lidars','bridge'}:
        errors.append("top-level contains unknown keys")
    if errors:
        raise ConfigError("; ".join(errors))

# --------------------------
# Adapter implementation
# --------------------------

def _default_bridge_exe() -> str:
    here = Path(__file__).resolve().parent
    exe = here / "bridge" / "build" / "livox_bridge"
    return str(exe)

@dataclass
class LivoxAdapterConfig:
    lidar_ip: str = os.getenv("LIVOX_LIDAR_IP", "10.0.0.10")
    host_ip: str = os.getenv("LIVOX_HOST_IP", "0.0.0.0")
    cmd_port: int = 56000
    point_port: int = 57000
    imu_port: int = 58000
    bridge_exe: str = os.getenv("LIVOX_BRIDGE_EXE", _default_bridge_exe())
    udp_listen_port: int = int(os.getenv("LIVOX_UDP_PORT", "18080"))
    use_udp: bool = True
    max_frames: int = 60
    max_stats: int = 300
    config_path: Path = Path(os.getenv("MID360_CONFIG_PATH", str(CONFIG_PATH_DEFAULT)))

@dataclass
class LivoxMid360Adapter:
    cfg: LivoxAdapterConfig
    _bridge_proc: Optional[subprocess.Popen] = field(default=None, init=False)
    _frames: Deque[Dict[str, Any]] = field(default_factory=lambda: deque(maxlen=60))
    _stats: Deque[Dict[str, Any]] = field(default_factory=lambda: deque(maxlen=300))
    _clients: List[WebSocket] = field(default_factory=list)
    _udp_task: Optional[asyncio.Task] = field(default=None, init=False)
    _stdout_task: Optional[asyncio.Task] = field(default=None, init=False)
    _stop_event: asyncio.Event = field(default_factory=asyncio.Event)
    _config: Dict[str, Any] = field(default_factory=dict, init=False)

    def _load_and_validate(self) -> None:
        if not self.cfg.config_path.exists():
            raise ConfigError(f"Config file not found: {self.cfg.config_path}")
        data = json.loads(self.cfg.config_path.read_text())
        validate_config(data)
        self._config = data

    def start(self) -> None:
        self._load_and_validate()
        if self._bridge_proc is not None:
            return
        args = [self.cfg.bridge_exe]
        # Bridge will read MID360_CONFIG_PATH itself; ensure env is set
        env = os.environ.copy()
        env['MID360_CONFIG_PATH'] = str(self.cfg.config_path)
        if self.cfg.use_udp:
            # Let bridge use per-device or global port per JSON; adapter listens on cfg.udp_listen_port
            pass
        else:
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
            raise RuntimeError(f"livox_bridge not found at {self.cfg.bridge_exe}. Build it as per README.")
        loop = asyncio.get_event_loop()
        if self.cfg.use_udp:
            self._udp_task = loop.create_task(self._run_udp_listener())
        else:
            assert self._bridge_proc is not None and self._bridge_proc.stdout is not None
            self._stdout_task = loop.create_task(self._run_stdout_reader(self._bridge_proc.stdout))

    def stop(self) -> None:
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

    async def _run_udp_listener(self) -> None:
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
                self._process_line(data.decode('utf-8', errors='ignore'))
        finally:
            sock.close()

    async def _run_stdout_reader(self, pipe) -> None:
        while not self._stop_event.is_set():
            line = await asyncio.get_event_loop().run_in_executor(None, pipe.readline)
            if not line:
                await asyncio.sleep(0.001)
                continue
            self._process_line(line)

    def _process_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            self._stats.append({"type": "log", "ts": time.time(), "msg": line})
            return
        mtype = msg.get('type')
        if mtype == 'frame':
            self._frames.append(msg)
            asyncio.create_task(self._broadcast(msg))
        else:
            self._stats.append(msg)

    async def _broadcast(self, payload: Dict[str, Any]) -> None:
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

    def latest_frame(self) -> Optional[Dict[str, Any]]:
        return self._frames[-1] if self._frames else None

    def recent_frames(self, count: int = 10) -> List[Dict[str, Any]]:
        if count <= 0:
            return []
        return list(self._frames)[-count:]

    def info(self) -> Dict[str, Any]:
        return {
            "config_path": str(self.cfg.config_path),
            "udp_listen_port": self.cfg.udp_listen_port,
            "frames_buffered": len(self._frames),
            "stats_buffered": len(self._stats),
            "bridge_running": self._bridge_proc is not None and self._bridge_proc.poll() is None,
        }

router = APIRouter(prefix="/lidar", tags=["lidar"])
_adapter: Optional[LivoxMid360Adapter] = None

def ensure_adapter() -> LivoxMid360Adapter:
    global _adapter
    if _adapter is None:
        cfg = LivoxAdapterConfig()
        _adapter = LivoxMid360Adapter(cfg)
        _adapter.start()
    return _adapter

@router.get("/info")
async def get_info() -> Dict[str, Any]:
    adapter = ensure_adapter()
    return adapter.info()

@router.get("/config")
async def get_config() -> Dict[str, Any]:
    adapter = ensure_adapter()
    return adapter._config

@router.post("/start")
async def start_bridge() -> Dict[str, Any]:
    adapter = ensure_adapter()
    adapter.start()
    return {"ok": True, "bridge_running": True}

@router.post("/stop")
async def stop_bridge() -> Dict[str, Any]:
    adapter = ensure_adapter()
    adapter.stop()
    return {"ok": True}

@router.get("/points/latest")
async def get_latest_frame() -> Dict[str, Any]:
    adapter = ensure_adapter()
    frame = adapter.latest_frame()
    if frame is None:
        raise HTTPException(status_code=404, detail="No frame yet")
    return frame

@router.get("/points/recent")
async def get_recent_frames(count: int = 10) -> List[Dict[str, Any]]:
    adapter = ensure_adapter()
    return adapter.recent_frames(count)

@router.websocket("/ws")
async def lidar_ws(ws: WebSocket) -> None:
    adapter = ensure_adapter()
    await ws.accept()
    adapter._clients.append(ws)
    try:
        while True:
            try:
                _ = await ws.receive_text()
            except WebSocketDisconnect:
                break
            except Exception:
                await asyncio.sleep(0.05)
    finally:
        try:
            adapter._clients.remove(ws)
        except ValueError:
            pass

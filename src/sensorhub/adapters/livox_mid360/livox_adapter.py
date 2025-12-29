
import os
import time
import socket
import struct
import select
import subprocess
import threading
import logging
from typing import Optional

from fastapi import APIRouter, HTTPException
from sensorhub.core.sensor_base import AbstractSensorAdapter

router = APIRouter(prefix="/livox", tags=["livox"])


@router.get("/service/status")
def livox_service_status(service_name: str = "livox_bridge"):
    try:
        result = subprocess.run(
            ["systemctl", "is-active", service_name],
            capture_output=True,
            text=True,
        )
        return {"service": service_name, "status": result.stdout.strip(), "code": result.returncode}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"systemd status failed: {e}")


@router.post("/service/start")
def livox_service_start(service_name: str = "livox_bridge"):
    try:
        subprocess.run(["sudo", "systemctl", "start", service_name], check=True)
        return {"service": service_name, "action": "start", "ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"systemd start failed: {e}")


@router.post("/service/stop")
def livox_service_stop(service_name: str = "livox_bridge"):
    try:
        subprocess.run(["sudo", "systemctl", "stop", service_name], check=True)
        return {"service": service_name, "action": "stop", "ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"systemd stop failed: {e}")


class LivoxMid360Adapter(AbstractSensorAdapter):
    def __init__(
        self,
        sensor_id: str,
        kind: str = "lidar3d",
        config_path: str = "/home/dev/livox_configs/mid360_config.json",
        use_systemd: bool = True,
        service_name: str = "livox_bridge",
        bridge_path: Optional[str] = None,
        multicast_ip: str = "224.1.1.5",
        point_port: int = 56301,
        imu_port: int = 56401,
        listen_udp: bool = True,
        publish_period: float = 0.5,
        hz: Optional[float] = None,    # <-- accept hz from config
        **kwargs,                      # <-- swallow any future keys safely
    ) -> None:
        super().__init__(sensor_id, kind)

        self.logger = getattr(
            self,
            "logger",
            logging.getLogger(f"sensorhub.adapters.livox_mid360.{self.__class__.__name__}.{sensor_id}"),
        )
        self._stop = getattr(self, "_stop", threading.Event())

        self.config_path = os.path.abspath(config_path)
        self.use_systemd = use_systemd
        self.service_name = service_name
        self.bridge_path = bridge_path
        self.multicast_ip = multicast_ip
        self.point_port = int(point_port)
        self.imu_port = int(imu_port)
        self.listen_udp = bool(listen_udp)

        # Map 'hz' (if provided) to publish_period, otherwise keep provided publish_period
        self.publish_period = (1.0 / hz) if (hz and hz > 0) else float(publish_period)

        self._pt_sock = None
        self._imu_sock = None
        self._proc = None

        self._point_pkts = 0
        self._point_bytes = 0
        self._imu_pkts = 0
        self._imu_bytes = 0
        self._last_point_ts = 0.0
        self._last_imu_ts = 0.0

        self._thread: Optional[threading.Thread] = None

    def _systemd_start(self) -> None:
        try:
            subprocess.run(["systemctl", "start", self.service_name], check=True)
            self.logger.info("Started systemd unit '%s'", self.service_name)
        except Exception as e:
            raise RuntimeError(f"systemd start failed for {self.service_name}: {e}")

    def _systemd_stop(self) -> None:
        try:
            subprocess.run(["systemctl", "stop", self.service_name], check=True)
            self.logger.info("Stopped systemd unit '%s'", self.service_name)
        except Exception as e:
            self.logger.warning("systemd stop failed for %s: %s", self.service_name, e)

    def _spawn_bridge(self) -> None:
        if not self.bridge_path:
            raise RuntimeError("bridge_path is required when use_systemd=False")
        if not os.path.isfile(self.bridge_path):
            raise RuntimeError(f"livox_bridge not found: {self.bridge_path}")
        if not os.path.isfile(self.config_path):
            raise RuntimeError(f"Livox config JSON not found: {self.config_path}")

        self._proc = subprocess.Popen(
            [self.bridge_path, self.config_path],
            cwd=os.path.dirname(self.bridge_path) or None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            universal_newlines=True,
        )
        self.logger.info("Spawned livox_bridge: %s %s", self.bridge_path, self.config_path)

    def _terminate_bridge(self) -> None:
        if self._proc:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3.0)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            finally:
                self._proc = None

    def _join_multicast(self, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("", port))
        except Exception as e:
            raise RuntimeError(f"Failed to bind UDP port {port}: {e}")

        mreq = struct.pack("4s4s", socket.inet_aton(self.multicast_ip), socket.inet_aton("0.0.0.0"))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.setblocking(False)
        self.logger.info("Joined multicast %s on UDP port %d", self.multicast_ip, port)
        return sock

    def start(self) -> None:
        if not os.path.isfile(self.config_path):
            raise RuntimeError(f"Livox config JSON not found: {self.config_path}")
        self.logger.info("Using Livox config: %s", self.config_path)

        if self.use_systemd:
            self._systemd_start()
        else:
            self._spawn_bridge()

        if self.listen_udp:
            try:
                self._pt_sock = self._join_multicast(self.point_port)
            except Exception as e:
                self.logger.warning("Point cloud listener setup failed: %s", e)
                self._pt_sock = None
            try:
                self._imu_sock = self._join_multicast(self.imu_port)
            except Exception as e:
                self.logger.warning("IMU listener setup failed: %s", e)
                self._imu_sock = None

        super().start()
        if not getattr(self, "_thread", None):
            self._thread = threading.Thread(
                target=self.run,
                name=f"LivoxMid360Adapter-{self.sensor_id}",
                daemon=True,
            )
            self._thread.start()

    def run(self) -> None:
        self.logger.info("Livox MID-360 run() loop started.")
        last_pub = time.time()
        try:
            while not self._stop.is_set():
                poll_end = time.time() + 0.05
                while time.time() < poll_end:
                    rlist = [s for s in (self._pt_sock, self._imu_sock) if s]
                    if not rlist:
                        break
                    rs, _, _ = select.select(rlist, [], [], 0.02)
                    for s in rs:
                        try:
                            data, _addr = s.recvfrom(65535)
                            now = time.time()
                            if s is self._pt_sock:
                                self._point_pkts += 1
                                self._point_bytes += len(data)
                                self._last_point_ts = now
                            else:
                                self._imu_pkts += 1
                                self._imu_bytes += len(data)
                                self._last_imu_ts = now
                        except Exception:
                            pass

                now = time.time()
                if (now - last_pub) >= self.publish_period:
                    payload = {
                        "sensor_id": self.sensor_id,
                        "status": "running",
                        "point_pkts": self._point_pkts,
                        "point_bytes": self._point_bytes,
                        "imu_pkts": self._imu_pkts,
                        "imu_bytes": self._imu_bytes,
                        "last_point_ts": self._last_point_ts,
                        "last_imu_ts": self._last_imu_ts,
                        "timestamp": now,
                    }
                    self.publish(payload)
                    last_pub = now

                time.sleep(0.01)
        except Exception as e:
            self.logger.error("Livox MID-360 run-loop error: %s", e)
        finally:
            self.logger.info("Livox MID-360 run() loop exiting.")

    def stop(self) -> None:
        for s in (self._pt_sock, self._imu_sock):
            if s:
                try:
                    s.close()
                except Exception:
                    pass
        self._pt_sock = None
        self._imu_sock = None

        if self.use_systemd:
            self._systemd_stop()
        else:
            self._terminate_bridge()

        super().stop()

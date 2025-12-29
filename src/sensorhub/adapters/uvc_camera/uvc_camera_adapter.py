
import time
import threading
import cv2
from typing import Optional, Union

from sensorhub.core.sensor_base import AbstractSensorAdapter


class UVCCameraAdapter(AbstractSensorAdapter):
    """USB UVC camera adapter using OpenCV.

    Publishes metadata to /sensors, and exposes latest JPEG bytes for /video endpoints.
    """

    def __init__(
        self,
        sensor_id: str,
        kind: str = "camera",
        device: Union[int, str] = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        quality: int = 80,
    ) -> None:
        super().__init__(sensor_id, kind)
        self.device: Union[int, str] = device
        self.width: int = width
        self.height: int = height
        self.fps: int = fps
        self.quality: int = int(max(1, min(100, quality)))
        self.latest_jpeg: Optional[bytes] = None
        self.frame_interval: float = 1.0 / max(1, fps)
        self._lock = threading.Lock()

    def run(self) -> None:
        cap = cv2.VideoCapture(self.device)
        # Try to set properties (not all cams honor these)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        seq = 0
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        while not self._stop.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            seq += 1
            # Encode JPEG
            ok_jpg, buf = cv2.imencode(".jpg", frame, encode_params)
            if ok_jpg:
                jpeg_bytes = buf.tobytes()
                with self._lock:
                    self.latest_jpeg = jpeg_bytes
                # Publish lightweight metadata (width/height/seq)
                self.publish(
                    {
                        "w": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        "h": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                        "seq": seq,
                    }
                )
            time.sleep(self.frame_interval)
        cap.release()

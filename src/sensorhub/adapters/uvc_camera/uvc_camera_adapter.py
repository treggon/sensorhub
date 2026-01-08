
# src/sensorhub/adapters/uvc_camera/uvc_camera_adapter.py
import time
import threading
import cv2
from typing import Optional, Union, Dict, Any

from sensorhub.core.sensor_base import AbstractSensorAdapter

# Optional import of Transform helper (works even if not yet added to your repo)
try:
    from sensorhub.core.transform import Transform
except Exception:
    Transform = None  # type: ignore


class UVCCameraAdapter(AbstractSensorAdapter):
    """USB UVC camera adapter using OpenCV.
    Publishes metadata to /sensors, and exposes latest JPEG bytes for /video endpoints.

    Updated:
    - __init__ now accepts 'transform: Optional[dict]' and '**kwargs', applies transform
      via adapter.set_transform() when present.
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
        transform: Optional[Dict[str, Any]] = None,
        **kwargs,
    ) -> None:
        super().__init__(sensor_id, kind)
        self.device: Union[int, str] = device
        self.width: int = width
        self.height: int = height
        self.fps: int = fps
        self.quality: int = int(max(1, min(100, quality)))
        self.latest_jpeg: Optional[bytes] = None
        self.frame_interval: float = 1.0 / max(1, fps)
        self._jpeg_lock = threading.Lock()
        self._seq = 0

        # Absorb any extra params without crashing (future-proof)
        for k, v in kwargs.items():
            setattr(self, k, v)

        # Apply transform if provided
        if transform:
            try:
                if Transform is not None:
                    self.set_transform(Transform.from_dict(transform))
                elif hasattr(self, "set_transform"):
                    self.set_transform(transform)  # type: ignore
                else:
                    setattr(self, "_transform", transform)
            except Exception:
                pass

    def run(self) -> None:
        cap = cv2.VideoCapture(self.device)
        # Try to set properties (not all cams honor these)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue

                self._seq += 1

                # Encode JPEG
                ok_jpg, buf = cv2.imencode(".jpg", frame, encode_params)
                if ok_jpg:
                    jpeg_bytes = buf.tobytes()
                    with self._jpeg_lock:
                        self.latest_jpeg = jpeg_bytes

                # Publish lightweight metadata (width/height/seq) -> sets ready=True on first publish
                self.publish(
                    {
                        "w": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        "h": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                        "seq": self._seq,
                        "fps": int(cap.get(cv2.CAP_PROP_FPS)),
                    }
                )

                time.sleep(self.frame_interval)
        finally:
            cap.release()

    # Optional: adapter-specific health
    def health(self) -> dict:
        h = super().health()
        with self._jpeg_lock:
            has_jpeg = self.latest_jpeg is not None
        h.update(
            {
                "device": self.device,
                "width": self.width,
                "height": self.height,
                "fps_cfg": self.fps,
                "jpeg_quality": self.quality,
                "seq": self._seq,
                "has_jpeg": has_jpeg,
            }
        )
        return h

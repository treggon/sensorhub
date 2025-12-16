
import serial
from sensorhub.core.sensor_base import AbstractSensorAdapter

class GenericIMUAdapter(AbstractSensorAdapter):
    """Generic IMU adapter that reads ASCII lines from a USB serial port and publishes them as text.
    """
    def __init__(self, sensor_id: str, port: str = '/dev/ttyUSB0', baudrate: int = 115200, kind: str = 'imu'):
        super().__init__(sensor_id, kind)
        self.port = port
        self.baudrate = baudrate

    def run(self):
        ser = serial.Serial(self.port, self.baudrate, timeout=1)
        while not self._stop.is_set():
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                self.publish({'imu_text': line})
        ser.close()

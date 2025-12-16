
import serial
from sensorhub.core.sensor_base import AbstractSensorAdapter

class UbloxGPSAdapter(AbstractSensorAdapter):
    """Simple u-blox adapter reading NMEA from /dev/ttyACM* or /dev/ttyUSB*.

    For UBX parsing, consider using pyubx2; this example emits raw NMEA lines for simplicity.
    """
    def __init__(self, sensor_id: str, port: str = '/dev/ttyACM0', baudrate: int = 9600, kind: str = 'gps'):
        super().__init__(sensor_id, kind)
        self.port = port
        self.baudrate = baudrate

    def run(self):
        ser = serial.Serial(self.port, self.baudrate, timeout=1)
        while not self._stop.is_set():
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                self.publish({'nmea': line})
        ser.close()


# SensorHub: Modular FastAPI/WebSocket sensor service

SensorHub is a modular, low-latency service for reading robot sensors directly from their SDKs or device outputs and serving the latest data via REST and WebSockets. It is designed to be expandable—add new sensors by dropping in adapters and adjusting a config file.

**Key features**
- ⚡ Low-latency async FastAPI backend with auto-generated Swagger/OpenAPI docs.
- 🔌 Pluggable sensor adapters (Livox Mid-360, RPLidar S2, u-blox GPS, IMU, Arducam USB cameras, plus a simulator).
- 🔒 TLS/mTLS support with self-signed or commercial certs.
- 🧵 Multi-threaded/async readers with per-sensor ring buffers and latest-value cache.
- 📡 WebSocket streaming and HTTP endpoints optimized for "latest sample" queries.
- 🧰 Unity client example included.

## Quick start (dev)
```bash
export SENSORHUB_CONFIG=src/sensorhub/config/config.min.yaml
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
uvicorn sensorhub.main:app --host 0.0.0.0 --port 8080
PYTHONPATH=$PWD/src uvicorn sensorhub.main:app --host 0.0.0.0 --port 8081
```
Then visit `http://localhost:8080/docs` for Swagger UI.

### TLS/mTLS
See `scripts/generate-selfsigned.sh` to generate dev certs, and configure the paths in `src/sensorhub/config/config.example.yaml`. Run uvicorn with `--ssl-certfile` and `--ssl-keyfile` (or via the provided script).

## Adding a sensor
1. Create a new folder under `src/sensorhub/adapters/<your_sensor>`.
2. Implement a class that inherits `AbstractSensorAdapter` (see `core/sensor_base.py`).
3. Add any SDK-specific notes in the adapter README.
4. Enable it in `config.yaml`.

## Production
- Run behind `gunicorn -k uvicorn.workers.UvicornWorker` and optionally behind Nginx.
- Consider enabling `uvloop` and `orjson` for performance.
- See `scripts/systemd/sensorhub.service` for a systemd example.


# SensorHub: Modular FastAPI/WebSocket Sensor Service

SensorHub is a modular, low-latency service for reading robot sensors (RPLidar S2/S3, u-blox GPS, Arducam USB cameras, IMU, and simulators) and serving their **latest sample** via REST and WebSockets. It is designed to be expandable—add new sensors by dropping in adapters and adjusting a config file.

## Features
- ⚡ Async FastAPI backend with auto-generated Swagger/OpenAPI docs
- 🔌 Pluggable sensor adapters (RPLidar, GPS, USB cameras, etc.)
- 🔐 TLS/mTLS support (self-signed or your internal CA)
- 🧵 Multi-threaded/async readers with per-sensor ring buffers
- 📡 WebSocket streaming and HTTP endpoints optimized for latest-value queries
- 🎮 Unity client example

---
## Quick start (developer)
```bash
# Serial devices (USB CDC ACM): dialout
sudo usermod -a -G dialout dev

# Some distros gate /dev/tty* also via 'tty' group; optional
sudo usermod -a -G tty dev

# Re-login so group membership applies (or reboot)
# On a headless shell, run:
newgrp dialout


# From the project root
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Point to a config (edit for your hardware)
export SENSORHUB_CONFIG=src/sensorhub/config/config.example.yaml
export SENSORHUB_CONFIG=src/sensorhub/config/config.some.yaml
# Run with PYTHONPATH so the src/ layout is importable
PYTHONPATH=$PWD/src uvicorn sensorhub.main:app --host 0.0.0.0 --port 8082

# to clean pycache
find src -name '__pycache__' -type d -exec rm -rf {} +
PYTHONPATH=$PWD/src uvicorn sensorhub.main:app --host 0.0.0.0 --port 8082


# HTTPS (self-signed)
./scripts/certs/generate-selfsigned.sh  # produces certs/server.crt and certs/server.key
PYTHONPATH=$PWD/src uvicorn sensorhub.main:app   --host 0.0.0.0 --port 8443   --ssl-certfile scripts/certs/server.crt   --ssl-keyfile scripts/certs/server.key
```

Visit `https://localhost:8443/docs` (accept the self-signed cert in browser).

---
## Systemd deployment (production-style)

Create `/etc/systemd/system/sensorhub.service`:

```ini
[Unit]
Description=SensorHub Service
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=sensorhub
WorkingDirectory=/opt/sensorhub
Environment=PYTHONPATH=/opt/sensorhub/src
Environment=SENSORHUB_CONFIG=/opt/sensorhub/src/sensorhub/config/config.example.yaml
ExecStart=/opt/sensorhub/.venv/bin/uvicorn sensorhub.main:app   --host 0.0.0.0 --port 8443   --ssl-keyfile /opt/sensorhub/certs/server.key   --ssl-certfile /opt/sensorhub/certs/server.crt
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```bash
# Provision
sudo useradd --system --create-home --shell /usr/sbin/nologin sensorhub
sudo mkdir -p /opt/sensorhub
sudo cp -r . /opt/sensorhub/
sudo chown -R sensorhub:sensorhub /opt/sensorhub

# venv at final location
sudo -u sensorhub /usr/bin/python3 -m venv /opt/sensorhub/.venv
sudo -u sensorhub /opt/sensorhub/.venv/bin/pip install --upgrade pip wheel
sudo -u sensorhub /opt/sensorhub/.venv/bin/pip install -r /opt/sensorhub/requirements.txt

# Permissions for devices
sudo usermod -aG dialout sensorhub   # serial (RPLidar, GPS)
sudo usermod -aG video sensorhub     # V4L2 cameras

# Enable & start
sudo cp scripts/systemd/sensorhub.service /etc/systemd/system/sensorhub.service
sudo systemctl daemon-reload
sudo systemctl enable --now sensorhub.service
journalctl -u sensorhub -f
```

---
## Troubleshooting

- **`ModuleNotFoundError: No module named 'sensorhub'`** → ensure `PYTHONPATH=/opt/sensorhub/src` in systemd, or install as a package.
- **TLS shows "Not secure"** → expected for self-signed certs. Reissue with SANs: `subjectAltName=IP:<robot-ip>,DNS:localhost` and/or trust an internal CA.
- **Serial `/dev/ttyUSB0` busy** → stop service and kill old uvicorn PIDs, add `RestartSec=3`, and handle hardware init with try/except so the service doesn’t crash.
- **Cameras 404** → pick the correct V4L2 node using `v4l2-ctl --list-devices`; many USB cams expose capture on `/dev/video1` rather than `/dev/video0`.

---
## Adding a new sensor

1. Create `src/sensorhub/adapters/<sensor_name>/` with an adapter class.
2. Register routes/endpoints in `src/sensorhub/routers/` (or the adapter’s setup).
3. Enable the adapter in your `config.yaml`.

---
## License
Apache-2.0 (or your company’s preferred license).

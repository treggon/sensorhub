
#!/usr/bin/env bash
set -euo pipefail
export SENSORHUB_CONFIG=src/sensorhub/config/config.example.yaml
uvicorn sensorhub.main:app --host 0.0.0.0 --port 8080

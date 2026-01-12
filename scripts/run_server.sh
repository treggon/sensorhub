cd ~/treggon/sensorhub
source .venv/bin/activate
export SENSORHUB_CONFIG=src/sensorhub/config/config.some.yaml
PYTHONPATH=$PWD/src uvicorn sensorhub.main:app --host 0.0.0.0 --port 8082 --log-level debug
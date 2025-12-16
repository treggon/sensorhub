
#!/usr/bin/env bash
set -euo pipefail
mkdir -p certs
# Generate a self-signed cert valid for 365 days
openssl req -x509 -nodes -newkey rsa:4096 -keyout certs/server.key -out certs/server.crt -days 365 -subj "/C=US/ST=CA/L=Robot/O=SensorHub/OU=Dev/CN=localhost"
# Optional: create a CA and sign client certs for mTLS in production workflows.

#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$REQUIRED_NEOS_VERSION" ]; then
  export REQUIRED_NEOS_VERSION="20"
fi

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="4"
fi

if [ -z "$PASSIVE" ]; then
  export PASSIVE="1"
fi

export STAGING_ROOT="/data/safe_staging"

# self-hosted backend (tools/logserver)
export API_HOST="https://openpilot.paratek.io"
# ATHENA_HOST intentionally unset -- tools/logserver has no websocket endpoint.
# Set it to wss://openpilot.paratek.io once that side exists.

# shared secret required by the self-hosted server to register a device.
# keep this out of git -- drop it in /data/params/d/ or a file the repo ignores.
if [ -f /data/registration_token ]; then
  export REGISTRATION_TOKEN="$(cat /data/registration_token)"
fi

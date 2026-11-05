#!/usr/bin/env bash
set -euo pipefail
TIANJI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${TIANJI_PYTHON:-}" ]]; then
    TIANJI_RUNTIME="$TIANJI_PYTHON"
elif [[ -x "$TIANJI_DIR/.venv/bin/python" ]]; then
    TIANJI_RUNTIME="$TIANJI_DIR/.venv/bin/python"
elif [[ -x "$TIANJI_DIR/../../work/tianji-venv/bin/python" ]]; then
    TIANJI_RUNTIME="$TIANJI_DIR/../../work/tianji-venv/bin/python"
else
    TIANJI_RUNTIME=python3
fi
exec "$TIANJI_RUNTIME" "$TIANJI_DIR/main.py" "$@"

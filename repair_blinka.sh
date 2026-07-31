#!/usr/bin/env bash
set -eu

if [ -z "${VIRTUAL_ENV:-}" ]; then
    echo "ERROR: Activate the Robot-Controller virtual environment first." >&2
    echo "Example: source .venv/bin/activate" >&2
    exit 2
fi

python -m pip uninstall -y board adafruit-pca9685 || true
python -m pip install --upgrade Adafruit-Blinka adafruit-circuitpython-pca9685

python - <<'PY'
import board

missing = [name for name in ("SCL", "SDA") if not hasattr(board, name)]
print(f"Imported board module: {getattr(board, '__file__', 'unknown')}")
if missing:
    raise SystemExit(
        "ERROR: board module still lacks: " + ", ".join(missing)
    )
print(f"SCL={board.SCL}, SDA={board.SDA}")
print("Adafruit Blinka board module is ready.")
PY

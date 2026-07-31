#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PYTHON="/home/robot/venvs/robot-controller/bin/python"

if [[ -x "${DEFAULT_PYTHON}" ]]; then
    PYTHON_BIN="${DEFAULT_PYTHON}"
else
    PYTHON_BIN="$(command -v python3)"
fi

cd "${SCRIPT_DIR}"
exec "${PYTHON_BIN}" ps2_manual_control.py "$@"

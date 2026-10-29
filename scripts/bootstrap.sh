#!/usr/bin/env bash
# Build isolated dependencies; downloads public pinned source into this workspace.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p vendor runtime
# MAVProxy's desktop windows use the distribution's wxPython/OpenCV packages.
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
if [[ ! -d vendor/jsbsim/.git ]]; then
  git clone --depth 1 --branch v1.3.3 https://github.com/JSBSim-Team/jsbsim.git vendor/jsbsim
fi
test "$(git -C vendor/jsbsim rev-parse HEAD)" = 66f7fafc07fe13e7ede9db20873030fed2881713
cmake -S vendor/jsbsim -B vendor/jsbsim/build -DBUILD_PYTHON_MODULE=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build vendor/jsbsim/build -j "${BUILD_JOBS:-6}"
if [[ ! -d vendor/ardupilot/.git ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/ArduPilot/ardupilot.git vendor/ardupilot
  git -C vendor/ardupilot checkout --detach bbf0b066e4
fi
test "$(git -C vendor/ardupilot rev-parse HEAD)" = bbf0b066e4baa04b5a746f22de642a25905cd52d
# Also finish submodule setup when retrying an interrupted installation.
git -C vendor/ardupilot submodule update --init modules/waf modules/mavlink modules/DroneCAN/DSDL modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard modules/DroneCAN/pydronecan
# Only build-tool compatibility changes; flight-controller source stays pinned.
.venv/bin/python - <<'PY'
from pathlib import Path
p=Path('vendor/ardupilot/modules/waf/waflib/Context.py')
p.write_text(p.read_text().replace('import os, re, imp, sys','import os, re, types, sys').replace('imp.new_module(WSCRIPT_FILE)','types.ModuleType(WSCRIPT_FILE)'))
PY
export PATH="$PWD/.venv/bin:$PATH"
cd vendor/ardupilot
./waf configure --board sitl --disable-tests --no-submodule-update
./waf plane -j "${BUILD_JOBS:-6}"

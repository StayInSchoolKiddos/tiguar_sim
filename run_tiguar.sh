#!/usr/bin/env bash
# Direct ArduPlane + JSBSim launch. No Python runner or MAVLink automation.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
tiguar_root="$PWD"
tiguar_model=tig_calibrated
if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: ./run_tiguar.sh [--model tig_calibrated|tig_da35|tig_da35_cruise]'
  echo 'Starts MAVProxy with map and console windows; aircraft disarmed on the ground.'
  echo 'Type exit in MAVProxy to stop both simulators.'
  exit 0
fi
if [[ $# -eq 2 && "$1" == '--model' ]]; then
  case "$2" in
    tig_calibrated|tig_da35|tig_da35_cruise) tiguar_model="$2" ;;
    *) echo 'Unknown Tiguar model; see --help.' >&2; exit 1 ;;
  esac
elif [[ $# -ne 0 ]]; then
  echo 'Invalid arguments; see --help.' >&2
  exit 1
fi
for binary in vendor/ardupilot/build/sitl/bin/arduplane vendor/jsbsim/build/src/JSBSim; do
  if [[ ! -x "$binary" ]]; then
    echo "Missing simulator: $binary. Run ./install_tiguar.sh first." >&2
    exit 1
  fi
done
tiguar_mavproxy="$tiguar_root/.venv/bin/mavproxy.py"
if [[ ! -x "$tiguar_mavproxy" ]]; then
  tiguar_mavproxy=$(command -v mavproxy.py || true)
fi
if [[ ! -x "$tiguar_mavproxy" ]]; then
  echo 'MAVProxy is missing. Run ./install_tiguar.sh first.' >&2
  exit 1
fi
# Prevent two copies of this launcher from sharing simulator ports.
exec 9>"$tiguar_root/runtime-launch.lock"
flock -n 9 || {
  echo 'Another Tiguar launcher is running. Type exit in its MAVProxy terminal.' >&2
  echo "To identify the process: fuser -v '$tiguar_root/runtime-launch.lock'" >&2
  exit 1
}
tiguar_work="$tiguar_root/runtime/tiguar_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$tiguar_work/aircraft"
cp -R "aircraft/$tiguar_model" "$tiguar_work/aircraft/"
cp config/sitl.parm "$tiguar_work/sitl.parm"
tiguar_overlay="config/models/$tiguar_model.parm"
if [[ -f "$tiguar_overlay" ]]; then
  awk 'NR==FNR { if (NF && $1 !~ /^#/) keys[$1]=1; next } !($1 in keys)' \
    "$tiguar_overlay" config/sitl.parm > "$tiguar_work/sitl.parm"
  cat "$tiguar_overlay" >> "$tiguar_work/sitl.parm"
fi
printf '\nINITIAL_MODE 0\n' >> "$tiguar_work/sitl.parm"
export PATH="$tiguar_root/vendor/jsbsim/build/src:$PATH"
export JSBSIM_DEBUG=0
cd "$tiguar_work"

echo "Model: $tiguar_model; results: $tiguar_work"
echo 'Starting MAVProxy: aircraft stays disarmed in MANUAL on the ground.'
echo 'Opening map and console windows (requires a desktop display).'
echo 'Type exit in MAVProxy to stop. Optional ground station: UDP port 14550.'
# JSBSim creates its own session: track and stop this launcher's children explicitly.
tiguar_pid=''
tiguar_children=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$tiguar_pid" && -r "/proc/$tiguar_pid/task/$tiguar_pid/children" ]]; then
    tiguar_children=$(cat "/proc/$tiguar_pid/task/$tiguar_pid/children")
  fi
  if [[ -n "$tiguar_children" ]]; then
    kill $tiguar_children 2>/dev/null || true
  fi
  if [[ -n "$tiguar_pid" ]]; then
    kill "$tiguar_pid" 2>/dev/null || true
    wait "$tiguar_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$tiguar_root/vendor/ardupilot/build/sitl/bin/arduplane" \
  -w \
  --model "jsbsim:$tiguar_model" \
  --home 38.6961834,-94.2578576,273,0 \
  --speedup 1 \
  --defaults "$tiguar_root/vendor/ardupilot/Tools/autotest/default_params/plane-jsbsim.parm,$tiguar_work/sitl.parm" \
  --autotest-dir "$tiguar_work" \
  --serial0 tcp:5760 \
  9>&- >console.log 2>&1 &
tiguar_pid=$!
# Wait for the backend before starting the interactive ground station.
tiguar_ready=0
for ((attempt=0; attempt<60; attempt++)); do
  if ! kill -0 "$tiguar_pid" 2>/dev/null; then
    echo 'ArduPilot exited during startup; see console.log.' >&2
    wait "$tiguar_pid"
    exit 1
  fi
  if [[ -r "/proc/$tiguar_pid/task/$tiguar_pid/children" ]]; then
    tiguar_children=$(cat "/proc/$tiguar_pid/task/$tiguar_pid/children")
  fi
  if (echo > /dev/tcp/127.0.0.1/5760) 2>/dev/null; then
    tiguar_ready=1
    break
  fi
  sleep 1
done
if [[ "$tiguar_ready" -ne 1 ]]; then
  echo 'Timed out waiting for ArduPilot; see console.log.' >&2
  exit 1
fi
"$tiguar_mavproxy" --master=tcp:127.0.0.1:5760 \
  --map --console \
  --out=udp:127.0.0.1:14550 --aircraft="$tiguar_work/mavproxy" \
  --cmd='set requireexit True; rc 3 1000' 9>&-

#!/usr/bin/env bash
# Direct ArduPlane + JSBSim launch. No Python runner or MAVLink automation.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
tiguar_root="$PWD"
if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: ./run_tiguar.sh'
  echo 'Connect your ground station via TCP to 127.0.0.1:5760.'
  echo 'Upload a mission and arm from the ground station. Ctrl-C stops the simulators.'
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo 'Use ./run_tiguar.sh without arguments; see --help.' >&2
  exit 1
fi
for binary in vendor/ardupilot/build/sitl/bin/arduplane vendor/jsbsim/build/src/JSBSim; do
  if [[ ! -x "$binary" ]]; then
    echo "Missing simulator: $binary. Run ./install_tiguar.sh first." >&2
    exit 1
  fi
done
# Prevent two copies of this launcher from sharing simulator ports.
exec 9>"$tiguar_root/runtime-launch.lock"
flock -n 9 || { echo 'Another Tiguar launcher is running.' >&2; exit 1; }
tiguar_work="$tiguar_root/runtime/tiguar_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$tiguar_work/aircraft"
cp -R aircraft/tig_calibrated "$tiguar_work/aircraft/"
cp config/sitl.parm "$tiguar_work/sitl.parm"
export PATH="$tiguar_root/vendor/jsbsim/build/src:$PATH"
export JSBSIM_DEBUG=0
cd "$tiguar_work"

echo "Results: $tiguar_work"
echo 'Connect ground station: TCP 127.0.0.1 port 5760'
echo 'Upload your mission and arm from the ground station. Ctrl-C stops the simulation.'
# JSBSim creates its own session: track and stop this launcher's children explicitly.
tiguar_pid=''
tiguar_children=''
cleanup() {
  trap - EXIT INT TERM
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
  --model jsbsim:tig_calibrated \
  --home 38.6961834,-94.2578576,273,0 \
  --speedup 1 \
  --defaults "$tiguar_root/vendor/ardupilot/Tools/autotest/default_params/plane-jsbsim.parm,$tiguar_work/sitl.parm" \
  --autotest-dir "$tiguar_work" \
  --serial0 tcp:5760 \
  >console.log 2>&1 &
tiguar_pid=$!
while kill -0 "$tiguar_pid" 2>/dev/null; do
  if [[ -r "/proc/$tiguar_pid/task/$tiguar_pid/children" ]]; then
    tiguar_children=$(cat "/proc/$tiguar_pid/task/$tiguar_pid/children")
  fi
  sleep 1
done
wait "$tiguar_pid"

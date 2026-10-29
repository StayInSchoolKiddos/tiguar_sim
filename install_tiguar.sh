#!/usr/bin/env bash
# One-time Ubuntu/Debian setup for this project's software-only simulator.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: ./install_tiguar.sh'
  echo 'Installs Ubuntu/Debian build prerequisites, then builds pinned ArduPilot and JSBSim.'
  echo 'Run as your normal user; sudo is used only for system packages.'
  echo 'Optional: BUILD_JOBS=2 ./install_tiguar.sh (reduce build memory use)'
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo 'Unexpected argument; see ./install_tiguar.sh --help.' >&2
  exit 1
fi
if [[ ! -f /etc/debian_version ]] || ! command -v apt-get >/dev/null; then
  echo 'This installer requires Ubuntu/Debian (including Ubuntu in WSL2).' >&2
  exit 1
fi
if [[ $EUID -eq 0 ]]; then
  echo 'Run ./install_tiguar.sh as your normal user, without sudo.' >&2
  exit 1
fi
if ! command -v sudo >/dev/null; then
  echo 'sudo is required to install system packages. Ask your administrator to enable it.' >&2
  exit 1
fi
trap 'echo "Setup failed. Fix the error above, then rerun ./install_tiguar.sh." >&2' ERR

echo 'Installing system prerequisites (sudo may ask for your password)...'
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ca-certificates git build-essential cmake pkg-config ccache \
  python3 python3-dev python3-venv python3-pip \
  python3-wxgtk4.0 python3-opencv python3-matplotlib python3-lxml \
  gawk rsync autoconf automake libtool \
  libxml2-dev libxslt1-dev zlib1g-dev util-linux

echo 'Installing project Python libraries and building ArduPilot + JSBSim...'
bash scripts/bootstrap.sh
test -x vendor/ardupilot/build/sitl/bin/arduplane
test -x vendor/jsbsim/build/src/JSBSim
echo 'Setup complete. Start the simulator with: ./run_tiguar.sh'

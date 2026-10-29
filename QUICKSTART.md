# Run the Tiguar simulation

Use an Ubuntu/Debian terminal (or Ubuntu in WSL2). No physical aircraft or autopilot is needed.

## 1. Open the project

Open a terminal in the project folder—the folder containing `run_tiguar.sh`. In VS Code, right-click that folder and choose **Open in Integrated Terminal**.

The project can live in any directory. The launcher finds the aircraft files and simulators relative to its own location; no fixed installation path is required.

## 2. Install once

```bash
./install_tiguar.sh
```

Run this as your normal user, without putting `sudo` in front. It asks for your password to install system prerequisites, then downloads and builds **ArduPilot and JSBSim** and installs the Python libraries used by the build and analysis tools. Internet access is required; the first build can take a while.

Wait for **Setup complete**. You can skip this step on later runs. A ground station is installed separately.

## 3. Start the simulators

```bash
./run_tiguar.sh
```

The shell script directly starts the compiled **ArduPilot** program, which starts **JSBSim** and loads the calibrated Tiguar. It runs at real-time speed. No Python runner is involved.

## 4. Use MAVProxy

MAVProxy connects automatically and opens a **map** and **console** window. The terminal accepts commands; the map shows position and the console shows vehicle status. Use a desktop session (or WSLg under WSL2) so the windows can open. Map imagery may need internet access.

The aircraft starts **disarmed, in MANUAL, on the ground**, with throttle held low. No mission is uploaded or started automatically.

At the MAVProxy prompt you can run:

```text
status
mode manual
rc 3 1000
```

Useful controls: `mode fbwa` selects stabilized manual flight, `mode rtl` requests return to launch, and `disarm` disarms after you are back on the ground. `rc 1 1500`, `rc 2 1500`, and `rc 4 1500` center roll, pitch, and rudder inputs; throttle is `rc 3 1000` (low) through `rc 3 2000` (high). These are simulated RC inputs, not keyboard joystick controls.

When ready to fly, load your mission with `wp load /path/to/mission.waypoints`, check it with `wp list`, then select `mode auto` and `arm throttle`. Use a fixed-wing mission containing TAKEOFF, waypoint, and LOITER items. Arming in AUTO can start the mission; do this only when ready.

An optional graphical ground station on the same computer can listen on **UDP port 14550**, forwarded by MAVProxy. MAVProxy uses TCP port 5760 itself.

## 5. Stop

Type **`exit`** at the MAVProxy prompt. The launcher stops ArduPilot and JSBSim too. Ctrl-C is reserved for interrupting MAVProxy commands; use `exit` to finish the session.

Each launch creates a new folder under `runtime/`. Its location is printed at startup.

| File in the run folder | Contents |
|---|---|
| `logs/*.BIN` | ArduPilot flight log |
| `console.log` | Simulator messages for troubleshooting |

## Which aircraft files are used?

For an explanation of the XML sections, units, coefficients, and engine connections, see [Understanding the Tiguar JSBSim files](AIRCRAFT_XML.md).

The launcher uses `aircraft/tig_calibrated/`:

| File | Purpose |
|---|---|
| `tig_calibrated.xml` | Aircraft geometry, mass, controls, and aerodynamics |
| `Engine/empirical_power.xml` | Equivalent engine shaft-power source |
| `Engine/Tiguar_20x10_CCW.xml` | Propeller model |

Keep the whole aircraft folder together. The launcher copies it into each run folder automatically. You do not need to copy files into ArduPilot or launch JSBSim separately. The original `aircraft/tig/` is retained as a reference.

The model is provisional. **`config/sitl.parm` is for simulation only; do not load it onto real aircraft hardware.**

## If something goes wrong

- **Address already in use:** stop the previous simulation with Ctrl-C. Run only one instance at a time.
- **Missing simulator:** run `./install_tiguar.sh`, then retry `./run_tiguar.sh`.
- **Map/console fail to open:** rerun `./install_tiguar.sh` to install GUI libraries, and launch from a desktop terminal. The project environment exposes the system wxPython/OpenCV packages.
- **Build runs out of memory:** retry with `BUILD_JOBS=2 ./install_tiguar.sh`.
- **Other startup failure:** give the project maintainer the terminal error and the run folder's `console.log`.

Python is required by the one-time build tools, but not by this flight launcher. The separate `scripts/run_sitl.py` remains available for automated analysis runs; this launcher does not call it. Existing comparison plots are in [the trajectory report](reports/trajectories/index.html); launching a new flight does not regenerate that report.

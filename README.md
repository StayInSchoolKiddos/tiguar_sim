# Tiguar / ArduPilot / JSBSim validation

See [the aircraft XML guide](AIRCRAFT_XML.md) for the JSBSim file structure, units, control mapping, engine/propeller connections, and calibration assumptions.

**First time running the simulator?** Follow the [intern quick-start](QUICKSTART.md): launch with `./run_tiguar.sh`, use MAVProxy with the aircraft disarmed on the ground, and type `exit` to stop.

This workspace contains a reproducible simulation and a **provisional**, log-conditioned Tiguar model. The original `aircraft/tig` is unchanged. The candidate is `aircraft/tig_calibrated`.

Read [the validation report](reports/validation.html) before interpreting the candidate as flight validated. The report separates simulator functionality, agreement with May 1 flight data, and independent April 30 validation. Matching the autopilot's airspeed setpoint alone does not validate the airframe.

The [trajectory comparison plots](reports/trajectories/index.html) overlay real and simulated states, body rates, controller demands, actuator commands, and relative 2D/3D paths, with simulation-minus-real delta panels. A [16-page PDF](reports/trajectories/tiguar_real_vs_sim.pdf), individual PNGs, and [paired CSVs and difference metrics](analysis/trajectory_comparisons) are available. Four selections cover May 1 climb/loiter/descent and an April 30 held-out climb. These are event/window-aligned portions of different missions, **not identical-command replays**; command and wind differences contribute to the deltas.

Rebuild these plots with `MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/compare_trajectories.py`. The script extracts the simulation BIN if needed. To change selections, copy/edit `analysis/trajectory_comparisons/alignment.json` and pass it with `--config path/to/selections.json`. The alignment file records exact timestamps, initial states and coordinate transformations. No time warping, fitted delays, or additional smoothing is used.

## Run

The dependencies were built locally under `vendor/`. With the Python environment used for this project:

```bash
# Automatic takeoff and right loiter, then stop and save telemetry
python3 scripts/run_sitl.py --model tig_calibrated --name demo

# Full speed, altitude, and left/right loiter test
python3 scripts/run_sitl.py --model tig_calibrated --name profile_demo --profile --duration 620

# Keep flying until Ctrl-C; optional MAVProxy connection on tcp:127.0.0.1:5762
python3 scripts/run_sitl.py --model tig_calibrated --name interactive_demo --interactive --speedup 1
```

The harness starts the exact ArduPlane 4.5.6 commit recorded in the logs and JSBSim 1.3.3. It uses local loopback sockets and needs an environment that permits them. Each run copies its aircraft and parameter files to `runtime/<name>/`, uploads a software-only mission, and records console output, MAVLink telemetry, mission, and an ArduPilot BIN. Use a new name to preserve earlier runs. Ctrl-C stops both simulator processes.

`config/sitl.parm` is **simulation-only**. It remaps equivalent aileron/elevator/throttle/rudder to outputs 1–4, preserves relevant logged control gains, and disables hardware arming/failsafe checks. Do not load it onto an aircraft. Actual logged throttle is output 16; the extractor resolves function assignments and endpoints over time.

For a fresh Ubuntu/Debian environment, install system prerequisites and build both simulators as your normal user:

```bash
./install_tiguar.sh
source .venv/bin/activate
```

This downloads public pinned simulator sources and Python dependencies. The original installation used a local shared clone of `/home/jn89b/ardupilot`; retain that source checkout or bootstrap a fresh clone when moving the workspace. `config/tool_versions.json` records the versions tested. The only ArduPilot build dependency edit replaces Python's removed `imp.new_module` with `types.ModuleType` in Waf; flight-controller source is unchanged.

## Reproduce the analysis

Run from the workspace root:

```bash
python3 scripts/extract_logs.py '2026-04-30 20-41 - 3 flights.BIN' '2026-05-01 14-00 - 13 flights.BIN'
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/calibrate.py
python3 scripts/build_candidate.py
python3 scripts/trim_sweep.py
python3 scripts/identify_dynamics.py
python3 scripts/run_sitl.py --model tig --name baseline_clean
python3 scripts/run_sitl.py --model tig_calibrated --name candidate_final_scaled --profile --duration 620
python3 scripts/check_pipeline.py
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/report.py
```

`analysis/<date>/` contains synchronized 10 Hz samples, parameter histories, flight boundaries, ten-second maneuver windows, messages, and missions. Sensor samples older than 0.6 s are discarded; discrete flight modes and parameter values persist until changed. Data extraction uses primary sensor instance/core zero, which must be revisited if logs show a primary-lane change. The flight detector combines `STAT.isFlying`, airspeed and GPS validity; the resulting 3 + 13 flights match the filenames independently.

The May 1 windows fit a monotone throttle-to-shaft-power curve plus effective thrust scale and zero-lift drag. April 30 is held out. The fit excludes flaps and advance ratios outside the supplied propeller table. Induced drag, geometry and mass are assumptions/inputs, not jointly identified free parameters. Climb estimates use the approximate energy balance `m*g*climb = (thrust-drag-m*acceleration)*TAS`.

## Model assumptions and scope

- Takeoff mass is approximately **55 lb / 24.95 kg**, including landing gear, per the user. Fuel burn is not modeled by the equivalent electric shaft-power source.
- Manual drawing span is 3.888 m; 330 mm root and 205 mm tip chords give a provisional trapezoidal area of 1.041 m². Actual inboard planform and airfoil coordinates are still needed for a stronger aerodynamic model.
- Loaded CG is provisionally x=0.94 m, z=0 on the model datum. The seed's vertical CG offset caused excessive power-on pitching moment. Actual CG is unknown.
- Loaded roll/pitch inertias are 20.5/15.5 kg·m². Yaw is provisionally 34 kg·m²: the supplied 41.4 exceeds roll plus pitch and cannot describe the same mass configuration about the same origin. Test-fixture corrections are unknown.
- Effective roll/pitch/yaw moment functions are scaled with inertia to preserve the seed's approximate angular response. This is a provisional simulation adjustment, not an independently identified derivative set. An exploratory log regression is saved but was too weak to justify replacing these priors.
- Ground contacts now use a tricycle layout and 0.713 m track. Fore-aft positions, wheel height, friction, springs and damping remain approximate. Takeoff success is a smoke test; landing/ground handling is not validated.
- Equivalent elevator/rudder channels retain the seed travel/effectiveness conventions. The physical aircraft has ruddervators with ±28° travel and flaps. Calibration excludes flap deployment; landing and stall behavior are outside the validated scope.
- The propeller is an approximate 20×10. Low-power windmilling/negative thrust beyond the supplied coefficient table is unresolved. Electric boost usage is unknown.

`config/model_assumptions.json` and `config/calibration.json` contain the numerical choices. The report includes failed experiments and held-out errors; they are not hidden by the final controller behavior.

The native JSBSim engine-trim solver takes 0.5 s propulsion steps, which are unstable for this small rotor inertia. `trim_sweep.py` uses a larger rotor inertia in temporary **equilibrium-only** copies to make trim converge. This does not affect steady thrust/power; the actual coupled flight runs retain the model's rotor inertia. Trim failures are recorded, not replaced with invented values.

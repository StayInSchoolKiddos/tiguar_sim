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

All maintained Tiguar profiles (`tig_calibrated`, `tig_da35`, and `tig_da35_cruise`) use this shared parameter file through both launch paths, with any explicit model defaults from `config/models/<model>.parm` applied afterward. `tig_da35_cruise` now defaults to `TKOFF_THR_MAX=75`; this is the only model-specific departure from the BIN-derived base. Regenerate it with `python3 scripts/match_sitl_parameters.py`: it reads the May 1 BIN directly and takes the latest recorded value at or before 1111 seconds, the first takeoff. It matches 155 flight-control, navigation, takeoff, landing, mode and filter settings, including `SCALING_SPEED=23` and `TKOFF_ROTATE_SPD=17`. `config/sitl.audit.json` lists every imported value, the 29 simulator overrides, and every excluded parameter. Hardware calibration and real ruddervator channel assignments remain separate from equivalent simulation axes. Mission commands, aerodynamic coefficients and electric assistance are not supplied by this parameter snapshot.

Use `--log`, `--at-time` and `--output` on the parameter generator for a different snapshot. An explicit `--param-file` on `scripts/run_sitl.py` permits isolated experiments. Both launchers copy the resolved shared-plus-model configuration into a fresh run directory. An explicit `--param-file` replaces this resolution entirely. Saved historical runs keep their original configuration. Run `python3 scripts/validate_tiguar_parameters.py --run` to check all three profiles, verify loaded values from their new BIN logs and exercise the full coupled mission. The [pitch investigation](reports/da35_cruise_analysis/pitch_investigation.html) documents the original overshoot and controlled parameter/coefficient experiments.

The [climb-control comparison](reports/da35_cruise_analysis/climb_options.html) tests lower TECS climb rate, lower AUTO takeoff minimum pitch, a pitch limit and a 75% takeoff throttle cap. The 75% takeoff limit is now the approved default for `tig_da35_cruise`; the other options remain isolated experiments. The shared BIN-derived base is preserved. Recreate them with `python3 scripts/investigate_da35_climb.py --run`. The harness's `--takeoff-pitch` controls the AUTO mission command (default 12°), separately from the autopilot parameter snapshot. Use `python3 scripts/tune_da35_cruise.py --report-only --run-dir runtime/da35_default75_validation` to attach the current validated run to the main report without refitting the engine.

For a fresh Ubuntu/Debian environment, install system prerequisites and build both simulators as your normal user:

```bash
./install_tiguar.sh
source .venv/bin/activate
```

This downloads public pinned simulator sources and Python dependencies. The original installation used a local shared clone of `/home/jn89b/ardupilot`; retain that source checkout or bootstrap a fresh clone when moving the workspace. `config/tool_versions.json` records the versions tested. The only ArduPilot build dependency edit replaces Python's removed `imp.new_module` with `types.ModuleType` in Waf; flight-controller source is unchanged.

## Reproduce the analysis

### DA35 cruise-flight analysis and empirical tuning

The [cruise tuning report](reports/da35_cruise_analysis/index.html) compares the DA35 candidate with actual level-flight and steady-turn windows, including held-out results and within-window variation. A separate `aircraft/tig_da35_cruise/` candidate fits one mid-throttle response factor to May 1; April 30 is not used to fit it. The DA35 baseline retains its BIN-derived takeoff setting. Launch the cruise-tuned model interactively with `./run_tiguar.sh --model tig_da35_cruise`; its 75% takeoff limit is automatic. Running the launcher without arguments still selects `tig_calibrated`.

The report also overlays each day's first recorded takeoff with the saved coupled simulation: EAS, RPM, relative height, climb rate, pitch and gasoline throttle. Time zero is sustained throttle application, not liftoff. These are different missions, with recalled electric assistance and flap deployment in the real flights; the simulation has no electric boost or calibrated flap model. Redraw saved results with `python3 scripts/tune_da35_cruise.py --report-only`. The takeoff section includes downloadable samples and alignment provenance; its generator is `scripts/plot_da35_takeoff.py`.

```bash
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/tune_da35_cruise.py
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/test_da35_cruise.py
python3 scripts/run_sitl.py --model tig_da35_cruise --name da35_cruise_validation --profile --duration 620
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/tune_da35_cruise.py --run-dir runtime/da35_cruise_validation
```

`config/da35_cruise_tuning.json` records the fitted factor, source hashes, data split and checks. The factor changes the input to the existing carburetor linkage only; it is not a horsepower reduction. It is fully applied over 20–50% command, tapers to no correction below 10% and above 70%, and preserves idle and full throttle. Unchanged high-power behavior still needs separate work.

The selected cruise windows are flaps-up, 21–25 m/s EAS, normal flight stage, and at least 60 seconds after flight detection. The existing quality filters and propeller-domain checks still apply. This selection excludes obvious boosted takeoff/climb, but no measured boost-off signal is available. There are only two held-out windows, both at higher throttle than the fitting windows. Use the model as an empirical cruise approximation, not independent proof of thrust, fuel-burn or takeoff accuracy.

### DA35 piston candidate

The separate `aircraft/tig_da35/` model uses a native JSBSim piston engine and a CSV-fitted throttle-to-carburetor linkage. The [DA35 comparison report](reports/da35_calibration/index.html) includes engine sweeps, start/idle/shutdown checks, and standalone takeoff diagnostics. The normal launcher still selects the accepted equivalent-power model in `tig_calibrated`.

```bash
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/da35_calibration.py
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/test_da35.py
```

Edit the engine source in `config/DA35.xml`; regeneration copies it into the candidate's `Engine/DA35.xml`. The fit and assumptions are recorded in `config/da35_calibration.json`. `--fit-only` rebuilds the candidate without regenerating diagnostics. This workflow never promotes the default.

The local Tiguar manual confirms displacement, cylinder/stroke counts and the RPM range. The seed's 4.5 hp figure is **not verified**. Full-throttle tests exceed the manual's range; the highest CSV bin constrains its 90% midpoint, not the unmeasured 100% endpoint. Electric assistance, real propeller loading and flap behavior remain unresolved.

The DA35 candidate completed a coupled 620-second profile after tool access was restored. The report records matching-model checks and settled-flight results. Reproduce with:

```bash
python3 scripts/run_sitl.py --model tig_da35 --name da35_validation --profile --duration 620
python3 scripts/check_pipeline.py --model tig_da35 --run-dir runtime/da35_validation
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/da35_calibration.py --sitl-run runtime/da35_validation
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/compare_da35_logs.py
```

These existing pipeline checks cover physical invariants and settled flight tracking. They do not validate gasoline-engine fuel consumption or takeoff fidelity. The standalone takeoff plots use a simple pilot controller and must not be described as an ArduPilot result.

The user recalls electric boost during takeoff/initial climb only. The manual describes roughly 50% takeoff power assistance; exact output and intervals in these logs are unknown. The candidate remains gasoline-only. See [actual steady-flight comparisons](reports/da35_calibration/steady_logs.html) and `config/da35_evidence.json`; the 4.5 hp seed rating remains unverified despite successful access to the manufacturer's product page.

### Fleet-average cruise engine calibration

The [cruise calibration report](reports/cruise_calibration/index.html) contains before/after RPM and shaft-power plots, 21–25 m/s speed sensitivity, comparisons with the original logs, and fresh coupled-flight results. Its acceptance status records whether the candidate became the default.

```bash
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/test_cruise_calibration.py
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/cruise_calibration.py
```

The workflow averages the four columns in `rpm_cruise_by_aircraft.csv` equally, uses throttle-bin midpoints, and fits at provisional 23 m/s TAS with the median May 1 air density. It stages a separate model, runs JSBSim sweeps and a new 620-second SITL profile, and promotes the result only after the RPM and pipeline checks pass. Local simulator sockets must be available; stop other simulator sessions first.

The original power calibration is preserved in `config/log_calibration_baseline.json`. Candidate calibration and provenance are in `config/cruise_calibration.json`. A completed run can be rechecked using `--run-dir runtime/<run-name>`; the workflow verifies that its aircraft files and parameters match the candidate. Generated baseline/candidate aircraft folders are staging artifacts.

To generate only the cruise configuration without flight testing or changing the default:

```bash
MPLCONFIGDIR=/tmp/tiguar-mpl python3 scripts/calibrate.py --cruise-csv rpm_cruise_by_aircraft.csv
```

The three lowest throttle bins are outside the propeller coefficient domain at the nominal fleet RPM. Low-power response retains the baseline shape rescaled to the first supported point; 90–100% throttle holds the last measured-bin power constant. Neither region is validated by this fit. Fleet-average RPM agreement does not establish thrust accuracy, and the new curve can fit the original individual-flight logs less closely.

### Original log-fit analysis

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

The original sequence above recalibrates from the logs and replaces the default with that log-fit model. Run the fleet cruise workflow afterward to restore the fleet calibration. `scripts/report.py` describes the archived original runs; use the dedicated cruise report for the fleet update.

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

The [current default report](reports/da35_cruise_analysis/default75.html) compares the updated takeoff with actual data and checks five settled cruise phases plus a dedicated straight-level leg. Reproduce the latter with `python3 scripts/run_sitl.py --model tig_da35_cruise --name da35_default75_straight --cruise-leg-m 6000 --duration 180`, then regenerate its report with `python3 scripts/report_da35_default75.py`. These checks distinguish closed-loop tracking from the cached engine RPM comparisons at recorded flight conditions.

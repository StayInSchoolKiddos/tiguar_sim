# Understanding the Tiguar JSBSim files

Use [QUICKSTART.md](QUICKSTART.md) to run the simulator. This guide explains the aircraft model for someone reading or editing its XML. It describes the current candidate and the JSBSim 1.3.3 implementation included by this project.

## How the files connect

```text
run_tiguar.sh
  -> ArduPlane --model jsbsim:tig_calibrated
     -> JSBSim
        -> aircraft/tig_calibrated/tig_calibrated.xml
           -> Engine/empirical_power.xml
           -> Engine/Tiguar_20x10_CCW.xml
```

The launcher copies the aircraft folder into a new `runtime/` directory. ArduPilot starts JSBSim there and generates its startup/reset files. Edit the source aircraft folder for the next launch, not an old runtime copy.

| File | Role |
|---|---|
| [tig_calibrated.xml](aircraft/tig_calibrated/tig_calibrated.xml) | Main aircraft definition: mass, geometry, controls, forces, and moments |
| [empirical_power.xml](aircraft/tig_calibrated/Engine/empirical_power.xml) | Active engine: equivalent shaft-power source |
| [Tiguar_20x10_CCW.xml](aircraft/tig_calibrated/Engine/Tiguar_20x10_CCW.xml) | Active propeller: power absorption, thrust, and rotational inertia |
| `Engine/DA35.xml`, `Engine/DA35_electric.xml` | Retained alternatives; neither is selected by the current main XML |
| [reset.xml](aircraft/tig_calibrated/reset.xml) | Standalone initial conditions; ArduPilot overwrites the runtime copy when launching |

The `file="empirical_power"` and `file="Tiguar_20x10_CCW"` attributes select those XML files without the `.xml` extension. Changing an unused engine file has no effect.

## Reading the main XML

The root `<fdm_config>` identifies the model. Its `version="2.0"` is the aircraft configuration format, not the installed JSBSim version. `<fileheader>` contains descriptive metadata, not flight physics. Some inherited header wording predates this calibration; use the assumptions below for its validation status.

| Section | Meaning in this aircraft |
|---|---|
| `<metrics>` | Wing reference area, span, mean aerodynamic chord, and reference locations |
| `<mass_balance>` | Empty mass, empty-aircraft CG/inertia, and payload point masses |
| `<ground_reactions>` | Three wheel contacts, friction, suspension, and nosewheel steering limit |
| `<propulsion>` | Engine/propeller selection, placement/orientation, and fuel tank |
| `<system>` | A heading sensor property used by the model |
| `<flight_control>` | Converts normalized control commands into surface angles and engine power fraction |
| `<aerodynamics>` | Functions producing aerodynamic forces and moments |
| `<output>` | JSBSim properties recorded to `jsbsim-state.csv` at 20 Hz |

### Geometry, mass, and coordinate conventions

The current reference area is **1.0407 m²**, span **3.888 m**, and mean aerodynamic chord **0.2725 m**. `AERORP` is the aerodynamic reference point; its offset from the CG contributes to the resulting moment. `EYEPOINT` and `VRP` are reference locations, not additional masses.

JSBSim's structural coordinates for aircraft locations use **X aft, Y right, Z up**, measured from a consistent model datum. Body axes use **X forward, Y right, Z down**. These are different conventions: the wheel locations have negative structural Z because they are below the datum. The model's X coordinates are not automatically distances behind the wing leading edge.

Always preserve explicit units such as `M`, `KG`, `KG*M2`, `IN`, and `DEG`. Many runtime properties use imperial units even when XML inputs use SI: `qbar-psf` is dynamic pressure in pounds-force per square foot and `Sw-sqft` is area in square feet. Multiplying them gives pounds-force. A property ending in `-rad` takes radians, not degrees.

`emptywt` is approximately **24.3976 kg**. The **0.05 kg** payload and **0.5 kg** tank contents bring the initial total to **24.9476 kg (55 lb)**, including landing gear. Do not enter 55 lb as empty weight and then add those masses again. JSBSim combines these masses and their positions to calculate loaded CG and inertia. The generator compensates the empty-aircraft values to target loaded CG X=0.94 m, Z=0 and diagonal inertias approximately 20.5/15.5/34 kg·m².

The wheel contacts describe ground forces, not separate wheel masses. Their geometry and suspension remain provisional.

### Controls: commands become deflections

ArduPilot's JSBSim connector supplies normalized aileron, elevator, rudder, and throttle commands. In this project's simulator configuration, outputs 1–4 correspond to aileron, elevator, throttle, and rudder. This differs from the real aircraft's channel assignment.

Inside `<flight_control>`:

- `<summer>` adds commands and trim; `<clipto>` limits the result.
- `<aerosurface_scale>` maps the command into a surface position. The current surface ranges are approximately ±20 degrees, represented in radians.
- Opposing left/right aileron positions are generated. The roll-control moment currently reads `fcs/left-aileron-pos-rad`; the separate effective-aileron output is not that moment's input.
- The empirical propulsion channel maps throttle command to `fcs/throttle-pos-norm`, which here represents a fraction of maximum modeled shaft power.

These channels describe equivalent elevator/rudder controls, not a detailed physical ruddervator/servo mechanism. The current mapping has no identified servo lag or backlash model. ArduPilot supplies the autopilot logic; this XML supplies the aircraft response.

### Engine and propeller

The selected engine is an `<electric_engine>` with **2363.44 W** maximum modeled power. This is a convenient shaft-power source, not a claim that the real DA35 is electric. The main aircraft XML contains the fitted throttle-to-power lookup table; editing maximum power alone rescales that table's physical output.

The approximate fitted relationship is `power_W = 876*u + 1488*u²`, where `u` is normalized throttle. High-throttle values are extrapolated. Gasoline-engine idle, fuel consumption, throttle delay, and separate hybrid boost are not explicitly represented. The tank still contributes mass; the equivalent engine does not burn its fuel.

The propeller file contains:

- `diameter`: 20 inches; `ixx`: rotor inertia, explicitly in kg·m².
- Equal `minpitch`/`maxpitch`: fixed blade pitch. These are blade-angle settings, not the “10 inches” of a nominal 20×10 propeller designation.
- `C_THRUST` and `C_POWER`: dimensionless coefficient tables indexed by advance ratio `J = V/(n*D)`, where `n` is revolutions per second.
- `ct_factor`: fitted thrust multiplier **1.06423**.

Conceptually, thrust is `Ct * rho * n² * D⁴` and absorbed power is `Cp * rho * n³ * D⁵`. JSBSim combines shaft power, propeller load, and rotational inertia to evolve RPM. These tables are approximate and stop near J=0.667; low-power windmilling is not validated.

**Rotation-direction caveat:** the filename's `CCW` text does not configure rotation. JSBSim uses `<sense>` under the main XML's `<thruster>`: +1 is clockwise viewed from the cockpit, −1 is counterclockwise. The current file omits it, so JSBSim defaults to +1. Confirm the actual installation before changing this; rotation affects torque/gyroscopic effects. This guide does not change the model's existing behavior.

### Aerodynamics: XML expressions calculate forces and moments

Each `<axis>` sums its `<function>` contributions. `DRAG`, `SIDE`, and `LIFT` supply wind-axis force components; `ROLL`, `PITCH`, and `YAW` supply moments. JSBSim handles the coordinate transformations and reference-point effects.

For example, the current base-drag expression is:

```xml
<function name="aero/coefficient/CD0">
  <product>
    <property>aero/qbar-psf</property>
    <property>metrics/Sw-sqft</property>
    <value>0.007900875984936018</value>
  </product>
</function>
```

This means **D₀ = q × S × CD₀**. Despite its name, the function returns a force in pounds-force, not the dimensionless coefficient alone. Likewise, the logged `aero/coefficient/Cm...` functions include scaling and return moment contributions.

`<property>` reads a simulation variable, `<value>` supplies a constant, `<product>` multiplies its children, and `<table>` interpolates data using its `<independentVar>`. In `CLalpha`, table columns are angle of attack in radians and lift coefficient. This is an assumed whole-aircraft lift curve, not an airfoil-coordinate file or a CFD calculation.

| Function group | Physical role |
|---|---|
| `CD0`, `CDi` | Base drag and induced drag; induced term uses `k * CL²`, with k=0.04 |
| `CDbeta`, `CDde` | Additional drag from sideslip and squared elevator deflection |
| `CLalpha`, `CLde` | Lift from angle of attack and elevator deflection |
| `CYb` | Side force due to sideslip |
| `Cl...` | Roll stability, damping, and control effectiveness |
| `Cm...` | Pitch trim, stability, damping, and elevator effectiveness |
| `Cn...` | Yaw stability, damping, and control/coupling effects |

Force functions commonly include `q*S`. Roll/yaw moments include `q*S*span`; pitch moments include `q*S*chord`. Rate-dependent terms use `aero/bi2vel` (span divided by twice airspeed) or `aero/ci2vel` (chord divided by twice airspeed) to normalize angular rates.

The final multipliers of approximately **4.415 (roll), 3.780 (pitch), and 7.411 (yaw)** scale the inherited moment functions alongside the provisional inertias. They preserve approximate seed-model angular response; they are not independently measured aerodynamic coefficients.

## What is fitted, and what is assumed?

Base drag, the thrust multiplier, and throttle-to-power mapping were fitted to selected flight-log windows near 23 m/s. Induced drag, most lift/moment derivatives, CG, yaw inertia, and parts of the landing gear remain assumptions. The airfoil document informs geometry; it does not automatically generate the lift/drag/moment tables. See [calibration.json](config/calibration.json) and [model_assumptions.json](config/model_assumptions.json).

## Editing without losing your changes

1. Keep `aircraft/tig/` as the original reference. Make experimental changes to the candidate and record what changed.
2. For reproducible changes, update [build_candidate.py](scripts/build_candidate.py) and/or its [calibration inputs](config/calibration.json). Running the generator overwrites candidate XML edits. Some values are hard-coded in that generator rather than read from JSON.
3. Start a new run so the launcher copies the updated model. Check `console.log` for loading errors and compare the new flight behavior with the baseline.
4. Change one physical assumption at a time. Loading successfully does not establish aerodynamic accuracy, especially for stall, landing, or engine transients.

## JSBSim source references

These links point to the exact source version used here. Its header comments document the XML interfaces; implementation files resolve defaults and units.

- [Aircraft loading and configuration](https://github.com/JSBSim-Team/jsbsim/blob/66f7fafc07fe13e7ede9db20873030fed2881713/src/FGFDMExec.h)
- [Mass balance and coordinate conventions](https://github.com/JSBSim-Team/jsbsim/blob/66f7fafc07fe13e7ede9db20873030fed2881713/src/models/FGMassBalance.h)
- [Aerodynamic functions and axes](https://github.com/JSBSim-Team/jsbsim/blob/66f7fafc07fe13e7ede9db20873030fed2881713/src/models/FGAerodynamics.h)
- [Function expressions](https://github.com/JSBSim-Team/jsbsim/blob/66f7fafc07fe13e7ede9db20873030fed2881713/src/math/FGFunction.h)
- [Propeller XML, coefficients, and rotation convention](https://github.com/JSBSim-Team/jsbsim/blob/66f7fafc07fe13e7ede9db20873030fed2881713/src/models/propulsion/FGPropeller.h)
- [Propeller implementation and defaults](https://github.com/JSBSim-Team/jsbsim/blob/66f7fafc07fe13e7ede9db20873030fed2881713/src/models/propulsion/FGPropeller.cpp)
- [Equivalent electric engine implementation](https://github.com/JSBSim-Team/jsbsim/blob/66f7fafc07fe13e7ede9db20873030fed2881713/src/models/propulsion/FGElectric.cpp)

The same files are available locally under `vendor/jsbsim/` after installation. The ArduPilot side of the connection is documented in the source at `vendor/ardupilot/libraries/SITL/SIM_JSBSim.cpp`.

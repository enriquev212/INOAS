# How To Run

This page contains the MATLAB/Simulink workflow for reproducing the INOAS
simulation modes.

## Requirements

- Tested with MATLAB R2026a.
- Simulink.
- Aerospace Blockset for the spacecraft dynamics model.
- Aerospace Toolbox and the MATLAB add-on `Ephemeris Data for Aerospace
  Toolbox`, required by the orbital environment blocks.
- Optimization Toolbox for the MPC optimization routine.

## Simulation Modes

| Mode | Script | MPC horizon `Np` | Simulink `StopTime` | Purpose |
| --- | --- | ---: | ---: | --- |
| Fast validation | `open_inoas_fast` | 25 | 120 s | Check installation, dependencies, model loading, and logging. |
| Debris demo | `open_inoas_debris_demo` | 25 | 950 s | Inspect the debris-avoidance maneuver around the configured encounter at 800 s. |
| Full run | `open_inoas_model` | 125 | 4000 s | Reproduce the final validation setup and plots. |

The full MPC setup uses a prediction horizon of `Np = 125`, so complete
simulations can take a long time on a laptop. Use `open_inoas_fast` first to
verify the MATLAB installation, Simulink dependencies, and logging workflow.
Use `open_inoas_debris_demo` when you want a shorter run that still reaches the
configured debris-avoidance encounter. Use `open_inoas_model` for the full
validation setup.

## Fast Validation

From the repository root in MATLAB:

```matlab
open_inoas_fast
out = sim("inoas_model");
```

## Debris-Avoidance Demo

From the repository root in MATLAB:

```matlab
open_inoas_debris_demo
out = sim("inoas_model");
run("matlab/plot_MPC_results.m");
```

## Full Validation Run

From the repository root in MATLAB:

```matlab
open_inoas_model
modelName = "inoas_model";
out = sim(modelName);
run("matlab/plot_MPC_results.m");
```

## Initialization

The helper scripts add the project folders to the MATLAB path, run
`initialize_inoas_simulation.m`, and open the top-level Simulink model. The
initialization step generates the nominal reference trajectory, debris
trajectory, GNSS quality profile, Kalman tuning, MPC parameters, and instrument
decision thresholds required by the model.

The main dataset is resolved through `matlab/inoas_data_file.m`, so scripts and
Simulink callbacks can refer to the GNSS `.dat` file by name while the file
remains organized under `data/`.

## Common Notes

If MATLAB reports missing ephemeris files such as `ephMoon405.mat`,
`ephEarthMoonBarycenter405.mat`, or `ephSun405.mat`, install the MATLAB support
package `Ephemeris Data for Aerospace Toolbox`.

Simulink may also warn that a few signal dimensions are inferred automatically.
This does not stop the simulation; it means Simulink resolved dimensions such as
the Kalman demux outputs during model compilation.

For presentation-style PNG/GIF generation after a simulation, see
[Visualization Workflow](visualization-workflow.md).

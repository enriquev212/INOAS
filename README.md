# INOAS | Integrated Navigation and Orbital Awareness System

**Autonomous satellite collision avoidance with energy-aware navigation.**
A team project developed at ISAE-SUPAERO for the **Student Aerospace
Challenge**, bringing together orbital simulation, state estimation, GNSS
receiver management and constrained maneuver planning in MATLAB/Simulink.

## What Our Team Built

Collision avoidance needs reliable navigation, while continuous GNSS operation
consumes power. We connected those decisions in one closed-loop demonstrator:

- **Navigation:** an Unscented Kalman Filter (UKF) combines intermittent GNSS
  information with auxiliary observations to estimate spacecraft position,
  velocity and uncertainty.
- **Receiver management:** a supervisor controls receiver operation using
  timing rules and navigation-quality checks, keeping receiver power separate
  from accepted navigation corrections.
- **Guidance:** Model Predictive Control (MPC) plans avoidance maneuvers with
  actuator limits and a safety margin informed by navigation uncertainty.
- **Integration and analysis:** a perturbed orbital plant, configurable runs,
  recorded settings, MATLAB/Python diagnostics and regression tests support
  further experiments.

**Tools:** MATLAB, Simulink, Aerospace Blockset/Toolbox, Optimization Toolbox,
Control System Toolbox and Python.

## Challenge Demonstrator

![Student Aerospace Challenge: debris-avoidance playback](docs/assets/debris-avoidance-playback.gif)

*Challenge demonstration: the planned trajectory departs from the uncorrected
encounter path while the control effort is shown alongside the motion. This
preserved animation uses the earlier Challenge configuration; it is not an
output of the current default CubeSat simulation.*

[Challenge poster](docs/assets/history/challenge-poster.pdf) |
[Full-quality presentation](https://github.com/enriquev212/INOAS/releases/download/inoas-project-materials-v1/INOAS_full_quality_final_presentation.pptx) |
[Challenge architecture and project history](docs/history.md)

## Try the Simulation

Requires MATLAB/Simulink and the [listed toolboxes](docs/how-to-run.md#requirements).
From the repository root in MATLAB, with the model closed:

```matlab
caseDir = run_inoas_case('reactive', 'adaptive');
```

This runs the current CubeSat demonstrator and exports a recorded case under
`results/`. For a short installation check, add `'StopTime', 120, 'Seed', 7`.
The current configuration has evolved from the Challenge demonstrator;
[the run guide](docs/how-to-run.md) describes its options and assumptions.

## Quick Links

| Explore | What you will find |
| --- | --- |
| [Architecture](docs/model-architecture.md) and [model](models/) | Current navigation, receiver and guidance implementation. |
| [Run guide](docs/how-to-run.md) and [examples](examples/README.md) | Setup, options and short demonstrations. |
| [MATLAB functions](matlab/README.md) and [GNSS data](data/README.md) | Code roles, input profiles and modeling assumptions. |
| [Diagnostics](docs/results.md) and [visualization](tools/visualization/README.md) | Interpret and plot your own simulation outputs. |
| [Development](docs/development.md) and [tests](tests/) | Continue the work and check changes. |
| [Project history](docs/history.md) and [references](docs/references.md) | Challenge materials and technical background. |

[Full documentation index](docs/README.md).

## Scope and Team

This is a simulation demonstrator, not flight-qualified software. The current
auxiliary observations are synthetic, GNSS inputs are processed profiles,
receiver power is modeled, and debris-state uncertainty is omitted. Its
constraints do not constitute an unconditional collision-safety guarantee.

Developed by the **Supaero Astra Iberian Team at ISAE-SUPAERO**: Alberto
Fernández-Acero Campoamor, Enrique Valverde Sacristán, Álvaro Yuste Pubill,
Guzmán Grande González, Júlia Soler i Pla and Changxiang Xu.
Maintainer: [Enrique Valverde](https://github.com/enriquev212).

[Software citation](CITATION.cff) | Code: [MIT](LICENSE).
Visual assets remain project-team materials unless separately authorized.

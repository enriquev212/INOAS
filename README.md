# INOAS | Integrated Navigation and Orbital Awareness System

GNSS receiver management and covariance-aware MPC collision avoidance for a
3U CubeSat-class spacecraft. Developed by the Supaero Astra Iberian Team at
ISAE-SUPAERO, combining orbital dynamics, UKF state estimation and constrained
optimal control in MATLAB/Simulink.

![INOAS debris-avoidance demonstration](docs/assets/debris-avoidance-playback.gif)

*Animation from the June 2026 Student Aerospace Challenge version, preserved
as a visual demonstration. The active model uses the paper's CubeSat setup;
see [project history](docs/history.md) for the animation's original context.*

## What INOAS Does

- **Manage receiver power:** Reactive duty cycling combines acquisition and
  timing rules with GNSS quality checks and an auxiliary discrepancy alarm.
- **Maintain navigation:** a UKF fuses intermittent GNSS updates, synthetic
  auxiliary position observations and orbital dynamics.
- **Adapt collision avoidance:** an MPC uses forecast navigation covariance
  to adjust its safety radius while respecting actuator constraints.

**Stack:** MATLAB, Simulink, Aerospace Blockset, UKF, nonlinear MPC and Python
visualization. [Architecture and paper diagrams](docs/model-architecture.md).

## Reported Paper Results

Medians over 50 paired runs with degraded GNSS, as reported in the submitted paper:

| Comparison | Result |
| --- | --- |
| Reactive vs continuous GNSS | 66.0% receiver-module energy saving |
| Reactive vs Fixed-Time, constant 295 m radius | Maximum position error: 8.0 m vs 21.1 m |
| Reactive/adaptive vs Reactive/constant radius | 32% lower applied Delta-v; adaptive median minimum separation: 194.2 m |

Every reported run stays above the 150 m keep-out distance. These are paper
results, not outputs reproduced by a single demo. The original campaign seed
list and complete outputs are not included; see [results](docs/results.md)
and [reproducibility limits](docs/model-provenance.md#reproducibility-limits).

## Quick Start

Requires MATLAB/Simulink and the toolboxes listed in [setup](docs/how-to-run.md#requirements).
From the repository root in MATLAB, with `inoas_model` closed:

```matlab
% Short installation check, with exported CSV/MAT results:
caseDir = run_inoas_case('reactive', 'adaptive', 'StopTime', 120);

% Full 6743 s scenario:
caseDir = run_inoas_case('reactive', 'adaptive');

% Or open the model for interactive work:
open_inoas_model;
```

The default is **Reactive + adaptive radius**. The runner also supports
`full` / `fixed` policies and `constant295`; run options include seed,
acquisition delay and initial estimation error. Results go into ignored
`results/` folders and are never overwritten.

[Run and compare cases](docs/how-to-run.md) |
[Short examples](examples/README.md) |
[MATLAB Online](docs/matlab-online.md)

## Work With the Project

| Location | Purpose |
| --- | --- |
| `run_inoas_case.m` / `open_inoas_model.m` | Batch and interactive entry points |
| `initialize_inoas_simulation.m` | Scenario, filter and MPC initialization |
| `models/` | Active integrated Simulink model |
| `matlab/` | Navigation, receiver logic and guidance functions |
| `examples/` | Short setup, navigation and encounter demos |
| `data/` | Versioned GNSS input profiles |
| `tools/` / `tests/` | Export/visualization tools and regression checks |
| `docs/` | Architecture, results, setup and research context |

Start with [the development guide](docs/development.md) to change a configuration,
run checks or resume research. [Documentation index](docs/README.md) collects
the detailed material. `main` is the active version; earlier models remain
available as [archive tags](docs/model-provenance.md#archived-development-branches).

## Team and Paper

The team submitted *Robust MPC-Based Collision Avoidance Guidance and Safe
Duty-Cycled GNSS Navigation for LEO CubeSats* to the **2027 IEEE Aerospace
Conference**. The abstract was accepted; full-paper review is pending.
[Paper configuration and citation](docs/conference.md) | [CITATION.cff](CITATION.cff)

Alberto Fernández-Acero Campoamor, Enrique Valverde Sacristán, Álvaro Yuste
Pubill, Guzmán Grande González, Júlia Soler i Pla and Changxiang Xu.

Maintainer: [Enrique Valverde](https://github.com/enriquev212)
([email](mailto:enriquevalverdesacristan@gmail.com)).

## License

Code: [MIT](LICENSE). Communication materials and visual assets under
`docs/assets/` remain project-team materials unless separately authorized.

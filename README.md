# Integrated Navigation and Orbital Awareness System (INOAS)

Shared public repository for the INOAS navigation and collision-avoidance
project, developed by the Supaero Astra Iberian Team for WP7: Reusable
Propulsion / Maintenance of the
[Student Aerospace Challenge 2025/2026](https://www.studentaerospacechallenge.eu/index.php/en).
The project was presented at the challenge final, Aerospace Challenge Day, at
Paris-Le Bourget on June 25, 2026, and was then extended into a paper for the
2027 IEEE Aerospace Conference.

INOAS studies how a LEO servicing spacecraft can reduce GNSS receiver duty cycle
while keeping enough navigation accuracy and collision-avoidance authority for
rendezvous and debris-avoidance operations. The implementation combines a
Simulink orbital plant, simulated GNSS measurements, UKF/Kalman state estimation,
instrument decision logic, and an MPC controller with covariance-aware safety
margins.

![Debris-avoidance playback from the challenge-final model](docs/assets/debris-avoidance-playback.gif)

*Debris-avoidance playback from the model presented at the challenge final
(June 2026).*

**Paper results** (IEEE Aerospace 2027; medians over 50 paired Monte Carlo runs
with degraded GNSS):

- Reactive receiver management saves 66.0% of the receiver-module energy
  relative to continuous GNSS operation.
- With the constant 295 m radius, the maximum position-estimation error drops
  from 21.1 m with fixed-time duty cycling to 8.0 m with reactive management,
  and is lower in all 50 paired runs.
- With reactive management and the covariance-adaptive safety radius, the
  minimum separation is 194.2 m and the applied Δv is 32% lower than with a
  constant 295 m radius. Every run stays outside the 150 m keep-out distance.

> **Code status.** `main` contains the September 2026 conference-adaptation
> model. The final paper uses a later configuration (encounter, MPC horizon,
> receiver supervisor, auxiliary sensors, noise settings, atmospheric drag and
> the Monte Carlo campaign, among others) that has not been merged into `main`
> yet, so the default runs in this repository do not reproduce the paper
> results.

## Project Materials

[Final poster PDF](docs/assets/final-poster-supaero-astra-iberian-team.pdf) |
[Final presentation PPTX](https://github.com/enriquev212/INOAS/releases/download/inoas-project-materials-v1/INOAS_full_quality_final_presentation.pptx)

The poster and presentation are the challenge-final material (June 2026). Their
figures come from the challenge model, a 10 t spacecraft, before the CubeSat
adaptation and the corrections made for the paper, and are superseded by the
paper results; see [Results](docs/results.md).

<details>
<summary>Poster preview</summary>

![Final INOAS poster](docs/assets/final-poster-preview.png)

</details>

## Core Idea

INOAS couples GNSS duty cycling with navigation uncertainty. A receiver
supervisor powers the GNSS receiver on a nominal ON/OFF schedule and accepts
fixes only after quality checks, while an auxiliary-sensor discrepancy alarm
(pseudo-NIS) can bring the next activation forward. In the paper's reactive
policy, poor tracking also triggers reacquisition and the alarm can extend
powered operation. Between GNSS fixes, the UKF propagates the state and
covariance and keeps fusing the auxiliary measurements.

The control layer uses that navigation confidence directly: the MPC inflates its
debris safety radius with the forecast navigation covariance, so avoidance
guidance becomes more conservative only when state knowledge is less certain.

## CubeSat Physical Model

The default setup on `main` keeps the existing Sentinel-6A-inspired INOAS
reference orbit and Sentinel-6A-derived GNSS-quality timing/profile data, while
replacing the original large-spacecraft physical assumptions with a
representative 3U CubeSat-class bus. The CubeSat bus and receiver assumptions
are inspired by the STF-1 duty-cycled GPS case from Lantto's CubeSat POD study:

- approximate 3U mass: `3 * 1.33 kg`;
- cross-sectional area: `0.03 m^2`, used by the propagated SRP model;
- drag coefficient: `2.2`, documented for completeness because atmospheric
  drag is not currently propagated;
- reflectivity coefficient: `1.0`, used by the propagated SRP model;
- representative dual-frequency receiver: NovAtel OEM615.

The translational actuation reference is kept separate from STF-1. For the
conference setup, the per-axis acceleration bound is derived from a
Seeker-class individual cold-gas thruster scale:

- optimizer box reference: `F_control = 0.10 N`;
- acceleration bound: `F_control / m_sat`, approximately `0.025 m/s^2`.

Seeker 1.0 is used as the actuator-architecture reference class: a NASA Johnson
Space Center 3U cold-gas free-flyer inspection demonstrator. The `0.10 N`
number is an acceleration-box reference, not a claim that the manoeuvre uses the
full box-limit thrust. CPOD is the flight-demonstrated 3U RPO reference: two
autonomous CubeSats with 3-DOF translational control that demonstrated
rendezvous and proximity operations on orbit.

The final paper keeps this platform, but takes the Pumpkin GPSRM 1 (NovAtel
OEM719) as GNSS module reference for its receiver power model and also
propagates atmospheric drag; see
[IEEE Aerospace 2027 paper](docs/conference.md#paper-configuration).

## Documentation

| Document | Why open it |
|---|---|
| [How to run](docs/how-to-run.md) | Simulink setup, simulation modes, dependencies and common MATLAB notes. |
| [Model architecture](docs/model-architecture.md) | System layers, navigation decision logic and covariance-aware safety equations. |
| [Navigation covariance prediction](docs/navigation-covariance-prediction.md) | UKF/MPC wiring changes, forecast assumptions, MATLAB tests, and CSV validation. |
| [Results](docs/results.md) | Paper results, superseded challenge-final figures, default parameters and generated plots. |
| [Visualization workflow](docs/visualization-workflow.md) | MATLAB-to-Python pipeline for regenerating PNG and GIF assets. |
| [IEEE Aerospace 2027 paper](docs/conference.md) | Paper configuration, how it differs from the code on `main`, and citation. |
| [MATLAB function index](matlab/README.md) | File-by-file guide to the MATLAB scripts and model helpers. |
| [References](docs/references.md) | Bibliography and external technical sources. |

## Quick Start

For the navigation-covariance trial in MATLAB Online, see
[MATLAB Online trial](MATLAB_ONLINE_TRIAL.md). From the extracted `INOAS`
folder, `run_navigation_trial` runs the 1000 s case and exports CSV/MAT files.

From the repository root in MATLAB:

```matlab
open_inoas_debris_demo
out = sim("inoas_model");
run("matlab/plot_MPC_results.m");
```

Use `open_inoas_fast` for a short setup check and `open_inoas_model` for the
full validation setup. See [How To Run](docs/how-to-run.md) for requirements,
simulation modes, and common MATLAB notes.

## Repository Layout

```text
.
|-- open_inoas_model.m
|-- open_inoas_fast.m
|-- open_inoas_debris_demo.m
|-- initialize_inoas_simulation.m
|-- models/
|-- matlab/
|-- tools/visualization/
|-- data/
|-- docs/
```

Key files:

- `models/inoas_model.slx` - final integrated Simulink model.
- `matlab/MPC_INOAS.m` - MPC tracking and debris-avoidance controller.
- `matlab/instrument_decision.m` - GNSS/Kalman mode-selection logic.
- `tools/visualization/` - optional workflow for regenerating PNG/GIF assets
  from a completed simulation.
- `docs/assets/` - architecture figure, poster preview, final poster PDF, and
  selected visual playback assets.

## Public Scope

This repository focuses on the shared public version of the INOAS navigation and
collision-avoidance work. It includes:

- the final integrated Simulink model and MATLAB helper scripts;
- curated scenario data and GNSS-quality inputs required by the runnable demos;
- technical documentation, selected result assets, poster material and the
  release-hosted final presentation;
- the MATLAB-to-Python visualization workflow used to regenerate selected PNG
  and GIF assets.

It does not include private team working history, internal drafts, uncurated raw
files, third-party reference PDFs without redistribution permission, or future
paper-review material unless separately authorized by the project team.

## Conference and Citation

The project was extended into the paper *Robust MPC-Based Collision Avoidance
Guidance and Safe Duty-Cycled GNSS Navigation for LEO CubeSats* for the **2027
IEEE Aerospace Conference** (Big Sky, Montana, March 6-13, 2027; session 12.01,
paper 2437). The abstract was accepted on **July 6, 2026**.

Citation details are available in [docs/conference.md](docs/conference.md) and
[`CITATION.cff`](CITATION.cff).

## Project Team

This repository is maintained as the shared public version of the team project.
The work was developed by the Supaero Astra Iberian Team (ISAE-SUPAERO students)
for the Student Aerospace Challenge.

Maintainer/contact: Enrique Valverde Sacristán
([enriquev212](https://github.com/enriquev212),
[enriquevalverdesacristan@gmail.com](mailto:enriquevalverdesacristan@gmail.com)).

Alberto Fernández-Acero Campoamor · Enrique Valverde Sacristán · Álvaro Yuste
Pubill · Guzmán Grande González · Júlia Soler i Pla · Changxiang Xu

## License

The source code is released under the MIT License; see [LICENSE](LICENSE).
Project communication materials, including the poster, final presentation, and
derived visual assets under `docs/assets/`, remain project-team materials unless
separately authorized.

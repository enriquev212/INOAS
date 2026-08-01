# Integrated Navigation and Orbital Awareness System (INOAS)

Shared public repository for the INOAS navigation and collision-avoidance
project, developed by the Supaero Astra Iberian Team for WP7: Reusable
Propulsion / Maintenance of the
[Student Aerospace Challenge 2025/2026](https://www.studentaerospacechallenge.eu/index.php/en).
The project was presented at the challenge final, Aerospace Challenge Day, at
Paris-Le Bourget on June 25, 2026.

INOAS studies how a LEO servicing spacecraft can reduce GNSS receiver duty cycle
while keeping enough navigation accuracy and collision-avoidance authority for
rendezvous and debris-avoidance operations. The implementation combines a
Simulink orbital plant, simulated GNSS measurements, UKF/Kalman state estimation,
instrument decision logic, and an MPC controller with covariance-aware safety
margins.

![Debris-avoidance playback](docs/assets/debris-avoidance-playback.gif)

**Headline results:** 82% GNSS energy reduction | below 40 m outage position
error | 483.2 m minimum debris separation vs 154 m robust safety radius.

## Project Materials

[Final poster PDF](docs/assets/final-poster-supaero-astra-iberian-team.pdf) |
[Final presentation PPTX](https://github.com/enriquev212/INOAS/releases/download/inoas-project-materials-v1/INOAS_full_quality_final_presentation.pptx)

<details>
<summary>Poster preview</summary>

![Final INOAS poster](docs/assets/final-poster-preview.png)

</details>

## Core Idea

INOAS uses uncertainty-driven GNSS duty cycling. When navigation covariance
remains below the selected threshold, GNSS is switched off and the estimator
propagates the state autonomously. When uncertainty grows, GNSS quality degrades,
or NIS indicates filter divergence, GNSS is reactivated and the navigation
solution is corrected.

The control layer uses that navigation confidence directly: the MPC increases
the debris safety margin when the estimated covariance grows, making avoidance
guidance more conservative when state knowledge is less certain.

## Documentation

- [How to run the Simulink model](docs/how-to-run.md)
- [Model architecture and technical scope](docs/model-architecture.md)
- [Results, generated plots, and key parameters](docs/results.md)
- [MATLAB-to-Python visualization workflow](docs/visualization-workflow.md)
- [Conference adaptation and citation](docs/conference.md)
- [References](docs/references.md)
- [MATLAB function index](matlab/README.md)

## Quick Start

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

## Conference and Citation

The project is being adapted into a paper for the **2027 IEEE Aerospace
Conference**. The abstract was accepted on **July 6, 2026** under paper number
**2437**.

Citation details are available in [docs/conference.md](docs/conference.md) and
[`CITATION.cff`](CITATION.cff).

## Project Team

This repository is maintained as the shared public version of the team project.
The work was developed by the Supaero Astra Iberian Team (ISAE-SUPAERO students)
for the Student Aerospace Challenge.

Maintainer/contact: Enrique Valverde Sacristán
([enriquev212](https://github.com/enriquev212),
[enriquevalverdesacristan@gmail.com](mailto:enriquevalverdesacristan@gmail.com)).

Alberto Fernandez-Acero Campoamor · Enrique Valverde Sacristán · Álvaro Yuste
Pubill · Guzmán Grande González · Julia Soler i Pla · Changxiang Xu

## License

The source code is released under the MIT License; see [LICENSE](LICENSE).
Project communication materials, including the poster, final presentation, and
derived visual assets under `docs/assets/`, remain project-team materials unless
separately authorized.

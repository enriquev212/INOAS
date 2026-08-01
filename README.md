# Integrated Navigation and Orbital Awareness System (INOAS)

Portfolio copy of the INOAS navigation and collision-avoidance work developed for the
Student Aerospace Challenge 2025/2026 by the Supaero Astra Iberian Team, WP7:
Reusable Propulsion / Maintenance.

The project studies how a LEO servicing spacecraft can reduce GNSS receiver duty
cycle while keeping enough navigation accuracy and collision-avoidance authority for
rendezvous and debris-avoidance operations. The final implementation combines a
Simulink orbital plant, simulated GNSS measurements, UKF/Kalman state estimation,
instrument decision logic, and an MPC controller with covariance-aware safety
margins.

![INOAS architecture](docs/assets/inoas-architecture.png)

Project downloads:

- [Final presentation PPTX](https://github.com/enriquev212/INOAS/releases/download/inoas-portfolio-materials-v1/INOAS_full_quality_final_presentation.pptx)
- [Final poster PDF](docs/assets/final-poster-supaero-astra-iberian-team.pdf)

<details>
<summary>Poster preview</summary>

![Final INOAS poster](docs/assets/final-poster-preview.png)

</details>

## Context

Future orbital servicing missions need precise autonomous navigation for docking,
rendezvous, and collision avoidance, but continuous GNSS operation increases power
consumption. INOAS explores a software-driven navigation architecture where GNSS is
activated only when the estimator needs it, while a Kalman/UKF propagation layer
keeps the vehicle state available during GNSS-off periods.

The validation uses Sentinel-6A precise orbit determination telemetry
(`Y24D011`) at approximately 1347 km altitude, with representative scenarios for
nominal tracking, GNSS signal outage, precision degradation, geometric drift, and
rendezvous.

## Conference Adaptation

The project is currently being adapted into a paper for the **2027 IEEE Aerospace
Conference**, held at the Yellowstone Conference Center in Big Sky, Montana,
March 6-13, 2027.

The abstract was accepted on **July 6, 2026**:

- **Title:** Robust MPC-Based Collision Avoidance Guidance and Safe Duty-Cycled
  GNSS Navigation for LEO CubeSats
- **Session:** 12.01 Orbital, Surface and Payload/Instrument Mission Operations
- **Paper number:** 2437
- **Full paper deadline:** October 2, 2026

For the conference version, the original Student Aerospace Challenge architecture
is being adapted toward a LEO CubeSat scenario. This includes scaling the physical
system and mission assumptions to better match CubeSat-class constraints while
preserving the main contribution: robust MPC-based collision avoidance coupled with
safe duty-cycled GNSS/UKF navigation.

## System Architecture

The simulation is organized around four functional layers:

1. **Scenario and data**
   - Reference orbital trajectory in ECI and LVLH/RTN frames.
   - Debris encounter scenario and propagated relative trajectory.
   - Real GNSS quality profile from Sentinel-6A telemetry.

2. **Truth and sensors**
   - Nonlinear spacecraft plant in Simulink.
   - Simulated GNSS sensor using realistic noise, time-varying covariance, number
     of visible satellites, PDOP, HPE, and VPE.
   - Internal sensors used during GNSS-off propagation.

3. **Estimation and decision**
   - UKF/Kalman estimator for continuous state and covariance propagation.
   - Instrument decision finite-state machine.
   - GNSS/Kalman navigation selector driven by NIS, GNSS health, and covariance
     growth.

4. **Control and outputs**
   - MPC controller for reference tracking and debris avoidance.
   - Covariance-aware safety margins for robust separation.
   - Plotting and diagnostics for tracking, NIS, innovation, duty cycle, Delta-V,
     and control effort.

## Core Idea

INOAS uses uncertainty-driven GNSS duty cycling. When the covariance remains below
the selected threshold, GNSS is switched off and the estimator propagates the state
autonomously. When uncertainty grows, GNSS quality degrades, or NIS indicates filter
divergence, the system reactivates GNSS and updates the navigation solution.

The MPC does not treat navigation and control as independent blocks: the estimated
covariance is used to increase the debris safety margin, so the controller becomes
more conservative when navigation confidence is lower.

## Main Results

From the final validation campaign:

- GNSS receiver energy use reduced by about **82%** compared with always-on GNSS.
- GNSS active time reduced to about **18%**, saving roughly **49 Wh/day**.
- UKF position error stayed below **40 m** during outages, with about **5 m** final
  error after a full GNSS outage scenario.
- MPC maintained a minimum debris separation of **483.2 m**, above the **154 m**
  robust safety radius.
- Peak control acceleration stayed below **0.018 m/s^2** across the RTN axes.
- Debris-avoidance maneuver cost was about **2.5 m/s Delta-V**, with **9.2 m/s**
  cumulative Delta-V over the 4000 s simulation.

These headline metrics are reported from the final project material. Scenario
plots are not stored as fixed outputs in this repository because the MPC response
depends on the selected setup, encounter geometry, tuning, and simulation horizon.

## Technical Scope

The technical work represented in this repository covers the navigation and
autonomy layer:

- Developing and integrating the GNSS simulation path using Sentinel-6A quality
  indicators and time-varying measurement covariance.
- Building the UKF/Kalman navigation architecture for GNSS-on and GNSS-off
  operation.
- Implementing and tuning instrument-decision logic based on GNSS health, NIS, and
  covariance growth.
- Connecting navigation confidence to the MPC debris-avoidance safety margin.
- Producing diagnostic scripts, validation plots, architecture diagrams, and the
  final project communication material.
- Adapting the system toward the ongoing LEO CubeSat conference-paper
  formulation.

This repository is a portfolio-oriented copy of a team project, prepared for public
CV sharing by the project contributors. Individual roles can be detailed separately
by each team member in their own CV, portfolio, or interview material.

## Repository Layout

```text
.
|-- Script_Concurso_Spacecraft_batch.m
|-- models/
|-- matlab/
|-- data/
|-- docs/assets/
```

Key files:

- `Script_Concurso_Spacecraft_batch.m` - initializes the MATLAB workspace before
  opening or simulating the Simulink model.
- `models/MPCcontrolledSpacecraft_plant_gnss_kalman_decisionFinal.slx` - final
  integrated Simulink model.
- `matlab/MPC_INOAS.m` - MPC tracking and debris-avoidance controller.
- `matlab/prepare_gnss_sensor_workspace.m` and
  `matlab/load_gnss_sensor_profile.m` - GNSS profile and measurement-noise setup.
- `matlab/instrument_decision.m` - GNSS/Kalman mode-selection logic.
- `matlab/README.md` - short description of each MATLAB function.
- `data/` - active telemetry and precomputed MATLAB data used by the simulation.
- `docs/assets/` - architecture figure, poster preview, and final poster PDF.

The main dataset is resolved through `matlab/inoas_data_file.m`, so scripts and
Simulink callbacks can refer to the GNSS `.dat` file by name while the file remains
organized under `data/`.

## How To Run

Requirements:

- MATLAB with Simulink.
- Aerospace Blockset for the spacecraft dynamics model.
- Optimization Toolbox for the MPC optimization routine.

Suggested workflow from the repository root:

```matlab
addpath(genpath(pwd));

modelName = "MPCcontrolledSpacecraft_plant_gnss_kalman_decisionFinal";
run("Script_Concurso_Spacecraft_batch.m");

open_system(modelName);
simOut = sim(modelName);
```

The initialization script generates or loads the reference trajectory, debris
trajectory, GNSS quality profile, Kalman tuning, MPC parameters, and instrument
decision thresholds required by the model.

## Technical Notes

- Reference and debris trajectories are expressed in orbital frames and transformed
  as needed for MPC tracking.
- The controller operates in a relative RTN/LVLH frame and uses a robust safety
  distance that grows with navigation uncertainty.
- GNSS duty cycling is represented by a selector variable `lambda`, where
  `lambda = 1` selects GNSS-updated navigation and `lambda = 0` selects propagated
  Kalman/UKF navigation.
- The estimator continues propagating during GNSS-off intervals, so control remains
  available even when GNSS is inactive or degraded.

## Future Work

- Complete the 6-DOF model with attitude-control integration.
- Extend the MPC to terminal docking operations.
- Connect the architecture to live multi-debris SSA tracking.
- Validate the navigation and control chain in hardware-in-the-loop.

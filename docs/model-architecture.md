# Model Architecture

## Context

Future orbital servicing missions need precise autonomous navigation for docking,
rendezvous, and collision avoidance, but continuous GNSS operation increases
power consumption. INOAS explores a software-driven navigation architecture where
GNSS is activated only when the estimator needs it, while a Kalman/UKF
propagation layer keeps the vehicle state available during GNSS-off periods.

The validation uses Sentinel-6A precise orbit determination telemetry (`Y24D011`)
at approximately 1347 km altitude, with representative scenarios for nominal
tracking, GNSS signal outage, precision degradation, geometric drift, and
rendezvous.

## Core Idea

INOAS uses uncertainty-driven GNSS duty cycling. When the covariance remains
below the selected threshold, GNSS is switched off and the estimator propagates
the state autonomously. When uncertainty grows, GNSS quality degrades, or NIS
indicates filter divergence, the system reactivates GNSS and updates the
navigation solution.

The MPC does not treat navigation and control as independent blocks: the
estimated covariance is used to increase the debris safety margin, so the
controller becomes more conservative when navigation confidence is lower.

## System Architecture

![INOAS architecture](assets/inoas-architecture.png)

The simulation is organized around four functional layers:

1. **Scenario and data**
   - Reference orbital trajectory in ECI and LVLH/RTN frames.
   - Debris encounter scenario and propagated relative trajectory.
   - Real GNSS quality profile from Sentinel-6A telemetry.

2. **Truth and sensors**
   - Nonlinear spacecraft plant in Simulink.
   - Simulated GNSS sensor using realistic noise, time-varying covariance,
     number of visible satellites, PDOP, HPE, and VPE.
   - Internal sensors used during GNSS-off propagation.

3. **Estimation and decision**
   - UKF/Kalman estimator for continuous state and covariance propagation.
   - Instrument decision finite-state machine.
   - GNSS/Kalman navigation selector driven by NIS, GNSS health, and covariance
     growth.

4. **Control and outputs**
   - MPC controller for reference tracking and debris avoidance.
   - Covariance-aware safety margins for robust separation.
   - Plotting and diagnostics for tracking, NIS, innovation, duty cycle,
     ΔV, and control effort.

## Technical Scope

The technical work represented in this repository covers the navigation and
autonomy layer:

- Developing and integrating the GNSS simulation path using Sentinel-6A quality
  indicators and time-varying measurement covariance.
- Building the UKF/Kalman navigation architecture for GNSS-on and GNSS-off
  operation.
- Implementing and tuning instrument-decision logic based on GNSS health, NIS,
  and covariance growth.
- Connecting navigation confidence to the MPC debris-avoidance safety margin.
- Producing diagnostic scripts, validation plots, architecture diagrams, and the
  final project communication material.
- Adapting the system toward the ongoing LEO CubeSat conference-paper
  formulation.

## Technical Notes

- Reference and debris trajectories are expressed in orbital frames and
  transformed as needed for MPC tracking.
- The controller operates in a relative RTN/LVLH frame and uses a robust safety
  distance that grows with navigation uncertainty.
- GNSS duty cycling is represented by a selector variable `lambda`, where
  `lambda = 1` selects GNSS-updated navigation and `lambda = 0` selects
  propagated Kalman/UKF navigation.
- The estimator continues propagating during GNSS-off intervals, so control
  remains available even when GNSS is inactive or degraded.

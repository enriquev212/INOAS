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

## Navigation and Decision Logic

The Simulink model separates the truth plant, simulated GNSS sensor, estimator,
navigation selector and MPC controller. The selector outputs the `lambda` flag,
where `lambda = 1` routes the GNSS-updated navigation solution and `lambda = 0`
routes the Kalman/UKF propagated solution.

At each decision step, the instrument-decision block checks GNSS health
indicators (`n_sat`, `PDOP`, `HPE`, `VPE` and fix validity), a covariance
observable `J`, and the duty-cycle timers. Here `J` is a normalized covariance
trace, implemented as `trace(S_inv * P * S_T_inv)`, used to force a return to
GNSS when propagation uncertainty grows too much. GNSS can be switched off after
its scheduled on-window, while Kalman/UKF propagation continues providing the
state estimate during GNSS-off intervals. GNSS is reactivated when the
propagation window expires or when the covariance observable exceeds its
threshold, provided the GNSS solution is healthy again.

Innovation and NIS diagnostics are logged alongside this selector state to
inspect filter consistency during degraded-GNSS and outage scenarios. The MPC
then receives the selected state and covariance, and uses that covariance to
inflate the debris-avoidance safety margin.

## Key Equations

The MPC safety margin is inflated with the relative-position covariance used in
the collision-avoidance frame:

```math
d_{\mathrm{safe},k}
=
d_0 + k_\sigma \sqrt{\lambda_{\max}\!\left(P_{r,k}\right)}
```

The debris-avoidance constraint then enforces robust separation at each
prediction step:

```math
\left\|r_{\mathrm{rel},k}\right\|_2
\ge
d_{\mathrm{safe},k}
```

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

## Scope Note

This document describes the model architecture and operating logic. For what the
public repository includes and excludes, see [Public Scope](../README.md#public-scope).

## Technical Notes

- Reference and debris trajectories are expressed in orbital frames and
  transformed as needed for MPC tracking.
- The controller operates in a relative RTN/LVLH frame and uses a robust safety
  distance that grows with navigation uncertainty.
- GNSS duty cycling is represented by the selector variable `lambda`, described
  in the navigation and decision logic section above.
- The estimator continues propagating during GNSS-off intervals, so control
  remains available even when GNSS is inactive or degraded.

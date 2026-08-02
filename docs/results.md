# Results and Parameters

## Headline Results

From the final validation campaign:

- GNSS receiver energy use reduced by about **82%** compared with always-on
  GNSS.
- GNSS active time reduced to about **18%**, saving roughly **49 Wh/day**.
- UKF position error stayed below **40 m** during outages, with about **5 m**
  final error after a full GNSS outage scenario.
- MPC maintained a minimum debris separation of **483.2 m**, above the **154 m**
  robust safety radius.
- Peak control acceleration stayed below **0.018 m/s²** across the RTN axes.
- Debris-avoidance maneuver cost was about **2.5 m/s ΔV**, with **9.2 m/s**
  cumulative ΔV over the 4000 s simulation.

These headline metrics are reported from the final project material. Scenario
plots are not stored as fixed outputs in this repository because the MPC response
depends on the selected setup, encounter geometry, tuning, and simulation
horizon.

![Debris-avoidance playback](assets/debris-avoidance-playback.gif)

## Generated Plots

`matlab/plot_MPC_results.m` generates:

- 3D orbital trajectory with reference, truth, estimated state, and debris path.
- X/Y/Z position tracking against the nominal reference.
- Cartesian position error and position-error norm.
- MPC applied control acceleration with actuator limits.
- Debris distance against the fixed and covariance-inflated safety radii.
- Dynamic safety radius used by the MPC over the prediction horizon.
- XY, XZ, and YZ trajectory projections.
- LVLH radial/tangential/normal tracking error with corresponding control
  effort.
- Cumulative maneuver ΔV and printed MPC performance summary.

For reproducible presentation-style assets, see
[Visualization Workflow](visualization-workflow.md).

## Key Parameters

| Parameter | Default value | Notes |
| --- | ---: | --- |
| MPC horizon `Np` | 125 | Full validation horizon; reduced to 25 by `open_inoas_fast` and `open_inoas_debris_demo`. |
| Reference/MPC sample time `h` | 3 s | Reference trajectory and MPC discretization step. |
| Sensor/estimator sample time `Ts` | 1 s | Main Simulink estimator and decision sample time. |
| Full-run `StopTime` | 4000 s | Default validation duration. |
| Debris encounter time `t_debris` | 800 s | Encounter inspected by the debris-demo mode. |
| Debris relative offset | `[50, 0, 0] m` | Closest-approach offset in the LVLH frame. |
| Debris relative velocity | `[0, 10, 0] m/s` | Tangential fly-by velocity in the LVLH frame. |
| Baseline safety radius `dsafe0` | 150 m | Enlarged by the MPC covariance-aware safety margin. |
| Control limit `u_max` | 0.05 m/s² | Per-axis acceleration bound. |
| Control-rate limit `du_max` | 0.007 m/s² per step | Per-axis command increment bound. |
| GNSS duty-cycle timers | 90 s / 10 s | GNSS-on and Kalman-propagation windows. |
| GNSS health thresholds | 4 satellites, PDOP 6, HPE/VPE 5 m | Used by the instrument-decision state machine. |

## Future Work

- Extend the validation set with additional debris geometries, GNSS-degradation
  profiles and covariance-threshold settings.
- Run Monte Carlo studies to quantify how navigation uncertainty, sensor
  quality and MPC tuning affect robust separation.
- Extend the current translational setup toward a 6-DOF spacecraft model with
  attitude dynamics, pointing constraints and attitude-control coupling.
- Prepare hardware-in-the-loop or onboard-prototype tests for the navigation
  decision logic and MPC timing behaviour.
- Continue adapting the formulation toward the IEEE Aerospace 2027 paper; see
  [Conference Adaptation](conference.md) for the CubeSat-oriented scenario and
  citation context.
- Improve runtime and logging workflows for the full-horizon MPC simulations,
  while keeping the fast and debris-demo modes useful for quick checks.

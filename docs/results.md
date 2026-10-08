# Results and Parameters

## Paper Results

The final IEEE Aerospace 2027 paper evaluates INOAS in a paired Monte Carlo
campaign: 50 replicates of a 6743 s close-encounter scenario with degraded GNSS,
each simulated in five configurations (250 runs). Within a replicate, the
initial estimation error and the actuator- and auxiliary-noise seeds are shared
by all five configurations. The table gives medians over the 50 runs.

| Configuration | Receiver energy [Wh] | Powered [%] | Position RMSE [m] | Max. position error [m] | Min. separation [m] | Applied Δv [m/s] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Full GNSS, adaptive radius | 3.465 | 100.0 | 0.71 | 3.12 | 162.8 | 2.202 |
| Fixed-Time, adaptive radius | 0.966 | 24.4 | 4.28 | 20.76 | 239.8 | 3.293 |
| Fixed-Time, constant 295 m | 0.966 | 24.4 | 4.24 | 21.06 | 295.2 | 3.635 |
| Reactive, adaptive radius | 1.177 | 29.9 | 2.32 | 7.91 | 194.2 | 2.479 |
| Reactive, constant 295 m | 1.177 | 29.9 | 2.34 | 7.96 | 294.7 | 3.661 |

Navigation errors are computed for t ≥ 135 s, after the common initialization
interval; the other metrics cover the complete run.

Main findings:

- Reactive receiver management keeps a median receiver-energy saving of 66.0%
  (P10-P90: 61.5-66.1%) relative to continuous GNSS. It uses about 22% more
  energy than fixed-time duty cycling, but, with the constant 295 m radius,
  lowers the median maximum position error from 21.06 m to 7.96 m, with a
  lower maximum error in every paired run.
- The covariance-adaptive safety radius lowers the median applied Δv by 32% with
  reactive operation and by 9% with fixed-time operation, relative to a constant
  295 m radius.
- The realized separation stays above the 150 m physical keep-out distance in
  every run (minimum 162.5 m).

Receiver energies use OFF 0.025 W, TRACKING 1.8 W and ACQUIRING 1.3 × 1.8 W, so
the savings are conditional on that power model. In a single matched
realization with the reactive policy and the adaptive radius, the saving stays
between 51.7% and 73.9% across the tested minimum acquisition delays (20, 35,
50, 70 and 120 s) and acquisition-to-tracking power ratios (1.0, 1.1 and 1.3).

The configuration behind these results is summarised in
[IEEE Aerospace 2027 paper](conference.md#paper-configuration). It has not been
merged into `main` yet, so the default runs in this repository do not reproduce
these results.

## Challenge-Final Figures (Superseded)

The figures presented at the challenge final in June 2026, and shown in the
poster and presentation (about 82% GNSS energy reduction and 483.2 m minimum
debris separation), were obtained with the challenge model: a 10 t spacecraft
with a different encounter and navigation setup, before the CubeSat adaptation
and the corrections made for the paper. They are superseded by the paper
results above.

![Debris-avoidance playback](assets/debris-avoidance-playback.gif)

*Debris-avoidance playback from the challenge-final model.*

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

## Default Parameters of the Code on `main`

These are the defaults of the September 2026 conference-adaptation model on
`main`. The final paper uses a different configuration; see
[IEEE Aerospace 2027 paper](conference.md#paper-configuration).

### Scenario and CubeSat Physical Model

| Parameter | Default value | Notes |
| --- | ---: | --- |
| Reference orbit | Sentinel-6A-inspired INOAS reference orbit | The orbital geometry remains the current INOAS scenario and is treated as the Sentinel-intended reference for the conference setup. |
| Semi-major axis `a` | 7714.43 km | Approximately 1336 km altitude. |
| Eccentricity `ecc` | 0.000095 | Near-circular reference orbit. |
| Inclination `inc` | 66.04 deg | Sentinel-6A-inspired INOAS reference orbit. |
| RAAN / argument of perigee / true anomaly | 116.6 / 90 / 131 deg | Current INOAS reference geometry. |
| Physical platform | STF-1-inspired 3U CubeSat | Bus-level assumptions from duty-cycled GPS CubeSat POD literature. |
| Approximate mass `m_sat` | 3.99 kg | Three CubeSat units at about 1.33 kg each. |
| Cross-sectional area `area` | 0.03 m² | STF-1 assumption; feeds the solar-radiation-pressure model propagated in the plant. |
| Drag coefficient `CD` | 2.2 | STF-1 assumption documented for traceability; atmospheric drag is not currently propagated in the guidance validation. |
| Reflectivity coefficient `ref` | 1.0 | STF-1 assumption; feeds the solar-radiation-pressure model propagated in the plant. |
| Solar radiation pressure | Active in plant | Propagated through the Simulink SRP block using `area`, `ref`, and `initMass`; the configured values give approximately `6.9e-8 m/s²`, or `2.6e-5 m/s` over the nominal 375 s MPC horizon. |
| Representative GNSS receiver | NovAtel OEM615 | Dual-frequency receiver used by STF-1. |
| Actuator architecture reference | NASA/JSC Seeker 1.0 | 3U cold-gas free-flyer inspection demonstrator used only to frame the actuator architecture and individual-thruster scale. |
| Flight-demonstrated RPO reference | CPOD | Two 3U CubeSats demonstrated autonomous RPO with 3-DOF translational control on orbit. |

### Guidance and Navigation

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
| Thrust-scale box reference `F_control` | 0.10 N | Seeker-class individual cold-gas thruster scale used to set the optimizer box, not reported effective manoeuvring thrust. |
| Control limit `u_max` | 0.0251 m/s² | Derived from `F_control/m_sat`; per-axis optimizer box bound. |
| Control-rate limit `du_max` | 0.007 m/s² per step | Per-axis command increment bound. |
| GNSS duty-cycle timers | 60 s / 300 s | Nominal GNSS-on and GNSS-off durations. |
| GNSS health thresholds | 4 satellites, PDOP 6, HPE/VPE 5 m | Used by the instrument-decision state machine. |

## Future Work

- Couple the translational guidance with attitude determination and control:
  attitude dynamics, pointing constraints, slew requirements and the
  directional realization of commanded thrust.
- Co-design the GNSS subsystem with the navigation supervisor, using lower-power
  acquisition and tracking, and making receiver activation depend on the
  navigation accuracy required for upcoming manoeuvres.
- Include debris-state uncertainty, further encounter geometries and GNSS
  degradation profiles, and higher-fidelity receiver power models.
- Assess the onboard computational load and extend the validation with broader
  Monte Carlo studies and hardware-oriented tests.

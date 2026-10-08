# Results and Parameters

## Paper Results

The submitted IEEE Aerospace 2027 paper reports a paired Monte Carlo
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

The configuration behind these results is summarized in
[IEEE Aerospace 2027 paper](conference.md#paper-configuration). The AUX3
model is now the default on `main`, but the original campaign inputs and seed
list are not included. A new default run is a demonstration, not a reproduction
of the ensemble medians. See [provenance](model-provenance.md).

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

## Current Run Diagnostics

Default parameters are listed once in
[paper configuration](conference.md#paper-configuration). The public runner
uses the AUX3 demo's fixed initial estimation error unless overridden.
Generic CSV errors include startup, unlike the reported table's t >= 135 s
evaluation window. Its minimum debris separation is sampled, not an exact
continuous-time closest-approach calculation.

Energy is integrated from the receiver's OFF/ACQUIRING/TRACKING occupancy,
not from the GNSS correction-enable flag. The commanded and applied Delta-v
are exported separately. See [metric definitions](../tools/visualization/README.md#metrics).

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

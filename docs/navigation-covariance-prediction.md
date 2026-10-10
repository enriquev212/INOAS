# Navigation Covariance Forecast

`predictNavigationCovarianceProfile.m` receives the current posterior UKF state,
covariance, current time, future sample times, previous held ECI acceleration
command and shared filter settings. It creates a separate nonlinear UKF; it
does not modify the online estimator.

## Forecast Assumptions

At each future 1 s step, the forecast predicts with the central-gravity/J2
state model and corrects with a synthetic auxiliary measurement chosen to
give zero innovation. The measurement dimension is three. Process covariance
includes the external acceleration-noise model and 10% per-axis execution
uncertainty on the held command. Alpha, beta, kappa and auxiliary covariance
match the shared settings of the online UKF.

No future GNSS corrections, future quality recovery, future auxiliary biases
or future alarm-triggered wakeups are predicted. The ECI command is the one
from the previous MPC update, since the forecast precedes optimization. Thus
the forecast is pessimistic with respect to GNSS availability, **not a
guaranteed bound** on future estimation errors or physical tracking uncertainty.

The MPC queries this 1 s covariance profile at its 12 s nodes and constructs
`dsafe0 + safetyCost * sqrt(lambda_max(P_position))`, with defaults 150 m and 3.
A factor of three is an uncertainty-inflation setting, not automatically a
99.73% three-dimensional coverage probability. Debris uncertainty is excluded.

## Between-Node Constraints

The candidate trajectory is reconstructed over **every** 12 s interval using
cubic Hermite interpolation. Each interval's minimum separation is compared
with the maximum radius on the 1 s forecast grid within that interval. A
violation greater than 0.01 m adds a supporting-plane constraint at the
fractional time, tangent along the candidate debris-to-spacecraft direction.
The problem is then re-solved without refining the 12 s control grid.

This numerical check and the soft-constraint optimizer do not establish a
formal guarantee under all model errors. The exported CSV minimum is a
sampled diagnostic, not the optimizer's continuous-time minimum calculation.

## Verification

```matlab
addpath(pwd, genpath('matlab'), genpath('tools'));
results = runtests('tests/test_navigation_prediction.m');
assertSuccess(results);
addpath('tests/integration');
run_navigation_smoke_test;
```

Tests cover agreement with a linear auxiliary-update Riccati recursion,
common-time invariance with respect to the MPC grid, the effect of execution
uncertainty, and finite positive orbital covariances. Receiver policies and
the input quality gate have separate tests under `tests/`.

The former scheduled-GNSS forecast and two-state model-rewiring helpers have
been removed from the public workflow. Current runs use the AUX3 six-argument
forecast directly; there is no need to rewire or save the Simulink model.

See [architecture](model-architecture.md), [run instructions](how-to-run.md)
and [provenance](model-provenance.md).

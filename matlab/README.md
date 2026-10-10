# MATLAB Function Index

The active model is `../models/inoas_model.slx`. The top-level runner
`../run_inoas_case.m` initializes, simulates and exports it; see
[How To Run](../docs/how-to-run.md).

| Function | Role |
| --- | --- |
| `inoasPaperConfig` | Validated public run options; Reactive/adaptive by default. |
| `inoasMinimalGnssConfig` | Shared 35/60/300 s Reactive timing and quality thresholds. |
| `inoasMinimalGnssStep` | Three-state Reactive supervisor and Full GNSS dispatch. |
| `inoasFixedGnssStep` | Fixed 96 s ON / 300 s OFF comparator. |
| `inoasMinimalGnssTick` | Fresh 3 s measurement-epoch test. |
| `inoasMinimalAuxScore` | Normalized post-update auxiliary residual score. |
| `inoasReceiverEnergy` | Left-held state occupancy and receiver-module energy. |
| `MPC_INOAS` | CW guidance, actuator bounds and node/between-node avoidance constraints. |
| `predictNavigationCovarianceProfile` | Auxiliary-only nonlinear UKF forecast. |
| `dynamicQukf` | Six-state process-noise covariance from unmodeled acceleration and command-dependent actuator uncertainty. |
| `myStateTransitionFcn` | Six-state central-gravity/J2 propagation. |
| `myMeasurementFcn` | Three-component auxiliary ECI position observation. |
| `gnss_measurement_fcn` | Six-component GNSS position/velocity observation. |
| `get_nominal_trajectory` | J2 reference generation. |
| `get_debris_trajectory` | J2 debris trajectory constructed for the design encounter. |
| `referenceFrameTransform` | ECI/RTN frame transformation. |
| `prepare_gnss_sensor_workspace` | Simulink input preparation. |
| `load_gnss_sensor_profile` | Reads processed GNSS profiles; initialization overrides smoothing with raw errors. |
| `load_gnss_quality_signals` | Quality-input preparation for the model callback. |
| `inoas_data_file`, `inoas_data_path` | Resolve versioned inputs under `../data/`. |
| `inoas_runtime_path` | Resolve generated files under `../results/cache/`. |
| `plot_MPC_results`, `get_logsout_signal` | MATLAB diagnostic plots and signal lookup. |

The historical two-state implementation is kept only in
[`tests/legacy/instrument_decision.m`](../tests/legacy/instrument_decision.m),
outside the active MATLAB directory. It is **not** the function executed by the
present Simulink receiver block; its block label alone does not identify the
algorithm. The regression tests add its directory temporarily.

Some estimator and sensor functions are embedded in the Simulink file. Their
settings and limitations are documented in [architecture](../docs/model-architecture.md)
and [data](../data/README.md). Do not treat unused legacy workspace variables
as evidence of the active measurement dimension or forecast method.

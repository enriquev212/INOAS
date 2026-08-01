# MATLAB Files

This folder contains the MATLAB functions used by the INOAS Simulink model,
initialization script, and result post-processing.

## Model Core

- `MPC_INOAS.m` - Model Predictive Control law for reference tracking and
  debris-avoidance guidance. It builds the prediction model, applies actuator and
  safety constraints, and returns the commanded control acceleration.
- `instrument_decision.m` - Finite-state navigation selector. It decides whether
  the system should use GNSS-updated navigation or Kalman propagation based on
  covariance, satellite visibility, PDOP, HPE, VPE, and solution validity.
- `myStateTransitionFcn.m` - State-transition model used by the Unscented Kalman
  Filter. It propagates the spacecraft state between measurement updates.
- `myMeasurementFcn.m` - Measurement function for the Kalman/UKF position
  observation model.
- `medida_gnss_fcn.m` - GNSS measurement function used by the Simulink sensor
  chain.

## Scenario Generation

- `get_nominal_trajectory.m` - Generates the nominal orbital reference trajectory
  used by the MPC.
- `get_nominal_trajectory_from_gnss.m` - Optional legacy generator that derives a
  replayed reference trajectory from the GNSS dataset.
- `get_debris_trajectory.m` - Builds the debris encounter trajectory relative to
  the reference orbit.
- `referenceFrameTransform.m` - Computes ECI-to-RTN/LVLH frame transformations
  used for relative tracking and collision-avoidance geometry.

## GNSS Data Preparation

- `prepare_gnss_sensor_workspace.m` - Prepares time-varying GNSS noise,
  covariance, validity, and quality signals for Simulink.
- `load_gnss_sensor_profile.m` - Reads the Sentinel-6A-derived `.dat` file and
  constructs the GNSS sensor profile.
- `load_gnss_quality_signals.m` - Loads GNSS quality indicators into the MATLAB
  base workspace before simulation.
- `inoas_data_file.m` - Resolves versioned input files stored under `../data/`.
- `inoas_data_path.m` - Returns the preferred `../data/` path for generated
  MATLAB data files.

## Post-Processing

- `plot_MPC_results.m` - Generates the main validation plots for trajectory
  tracking, debris separation, control effort, and navigation mode selection.
- `get_logsout_signal.m` - Helper used by `plot_MPC_results.m` to retrieve logged
  Simulink signals with compatible alternative names.

# MATLAB Files

This folder contains the MATLAB functions used by the INOAS Simulink model,
initialization script, and result post-processing.

## Main Execution Flow

1. `initialize_inoas_simulation.m` prepares the base workspace: input file
   paths, physical scenario, Kalman/UKF tuning, GNSS sensor profile, nominal
   reference trajectory, debris encounter, and MPC parameters.
2. The Simulink model reads those workspace variables and calls the estimator,
   instrument-decision logic, and MPC controller during simulation.
3. `plot_MPC_results.m` can be run after simulation to regenerate plots for the
   selected scenario setup.
4. `tools/visualization/export_visualization_data.m` can be run after simulation
   to export a compact `.mat` file for the optional Python visualization
   workflow.

## Model Core

- `MPC_INOAS.m` - Model Predictive Control law for reference tracking and
  debris-avoidance guidance. It builds the prediction model, applies actuator and
  safety constraints, and returns the commanded control acceleration.
- `instrument_decision.m` - GNSS supervisor using shared on/off durations,
  a heuristic residual score, satellite visibility, and GNSS quality. Exposes
  a status snapshot for the optional scheduled navigation forecast.
- `navigationDutyCycleStep.m` - Pure supervisor transition shared by the real
  state machine and the nominal forecast.
- `predictNavigationCovarianceProfile.m` - UKF posterior covariance forecast
  at the estimator rate, including auxiliary and optional scheduled GNSS updates.
- `myStateTransitionFcn.m` - State-transition model used by the Unscented Kalman
  Filter. It propagates the spacecraft state between measurement updates.
- `myMeasurementFcn.m` - Measurement function for the Kalman/UKF position
  observation model.
- `gnss_measurement_fcn.m` - GNSS measurement function used by the Simulink sensor
  chain.

## Scenario Generation

- `get_nominal_trajectory.m` - Generates the J2-propagated nominal orbital
  reference trajectory used by the MPC.
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

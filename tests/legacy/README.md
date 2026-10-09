# Historical Regression Fixtures

`instrument_decision.m` preserves the old two-state GNSS/Kalman supervisor for
regression tests. It is not used by `models/inoas_model.slx` and is deliberately
outside the active `matlab/` directory.

`test_gnss_quality_gate` temporarily adds this directory to the MATLAB path and
restores the previous path afterward. Do not add it to a simulation's path.
The active three-state supervisor is
[`inoasMinimalGnssStep`](../../matlab/inoasMinimalGnssStep.m).

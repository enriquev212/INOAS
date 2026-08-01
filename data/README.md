# Data Files

This folder contains the active data files used by the MATLAB/Simulink
simulation.

- `cov_perturb_POS_s6a_Y24D011_fixed.dat` - Sentinel-6A-derived GNSS quality
  and covariance dataset used by the navigation sensor model.
- `perturb_POS_s6a_Y24D011.dat` - original GNSS perturbation dataset retained
  for traceability and comparison with earlier processing.
- `referenceTrajectory.mat` - nominal J2-propagated reference trajectory used by
  the MPC guidance layer. The main setup script can regenerate this file.
- `debrisTrajectory.mat` - debris encounter trajectory used by the avoidance
  scenario. The main setup script can regenerate this file.

Simulation plots should be regenerated from the selected scenario setup rather
than treated as fixed repository outputs.

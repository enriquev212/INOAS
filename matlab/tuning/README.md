# MPC Delta-V tuning

This folder tunes only the three weight vectors defined in `initialize_inoas_simulation.m`:

- `Q_step` - state tracking penalty;
- `R_step` - absolute control penalty;
- `S_step` - control-increment penalty.

The outer tuning objective is total applied Delta-V over the simulation. A candidate is accepted as **functional** only if it keeps a non-negative covariance-aware debris margin, respects the actuator box, and preserves baseline-level post-encounter/final tracking accuracy.

From the repository root in MATLAB R2026a (with the same Simulink/Aerospace/Optimization dependencies used by INOAS):

```matlab
[best, results] = run_mpc_tuning;
```

For a smaller first campaign:

```matlab
opt = struct( ...
    'NumGroupCandidates', 10, ...
    'NumLocalCandidates', 16, ...
    'NumFullValidation', 3);
[best, results] = run_mpc_tuning(opt);
```

The tuner first measures the current baseline, searches group scalings in logarithmic space, refines the individual 12 weight components, and finally reruns the best candidates over the full horizon. Progress is checkpointed after every candidate.

Outputs are written to `tuning_results/`:

- `mpc_tuning_checkpoint.mat` - latest recoverable campaign state;
- `mpc_tuning_YYYYMMDD_HHMMSS.mat` - complete final result;
- `mpc_tuning_YYYYMMDD_HHMMSS.csv` - one row per tested candidate;
- `best_mpc_weights.m` - the best full-horizon functional `Q_step`, `R_step`, `S_step` values.

The script intentionally reads `Np`, `h`, `t_debris`, `dsafe0`, actuator limits, trajectories and all other scenario data from the current `initialize_inoas_simulation.m` workspace. It does not tune or override those quantities.

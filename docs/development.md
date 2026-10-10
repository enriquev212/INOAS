# Continuing Development

## Start From the Active Version

`main` contains the supported model. Record its commit as your baseline,
and create a branch for a new experiment.
Historical models are preserved in a
[maintainer-held archive](model-provenance.md#archived-development-branches).
Keep existing local edits before switching versions. In MATLAB, work from
the repository root and close the model before batch runs.

## Change Only What the Experiment Needs

For receiver policy, radius mode, acquisition delay, duration, seed and initial
error, use run options rather than editing the model:

```matlab
caseDir = run_inoas_case('reactive', 'adaptive', ...
    'AcquisitionTime', 50, 'StopTime', 1800, 'Seed', 43);
```

Use identical seeds and initial errors for paired comparisons. Acquisition
delay applies after every entry into ACQUIRING; it is separate from the
Fixed-Time ON/OFF calendar. See [run options](how-to-run.md#batch-runs).

For changes beyond those options:

| Area | Starting point |
| --- | --- |
| Default run options | [inoasRunOptions.m](../matlab/inoasRunOptions.m) |
| Physical scenario, UKF/MPC tuning and trajectories | [initialize_inoas_simulation.m](../initialize_inoas_simulation.m) |
| Receiver timing and quality gates | [inoasMinimalGnssConfig.m](../matlab/inoasMinimalGnssConfig.m) |
| Reactive / Fixed-Time switching | [inoasMinimalGnssStep.m](../matlab/inoasMinimalGnssStep.m) / [inoasFixedGnssStep.m](../matlab/inoasFixedGnssStep.m) |
| Guidance and safety constraints | [MPC_INOAS.m](../matlab/MPC_INOAS.m) |
| Navigation covariance forecast | [predictNavigationCovarianceProfile.m](../matlab/predictNavigationCovarianceProfile.m) |
| Block connections and embedded filter/sensor functions | [inoas_model.slx](../models/inoas_model.slx) |
| CSV/MAT exports and Python plots | [visualization tools](../tools/visualization/README.md) |

Keep `initialize_inoas_simulation.m` at the repository root: entry points and
model initialization rely on it. Do not add `tests/legacy/` to the active MATLAB
path. The historical two-state supervisor is a regression reference only.
Changing the MPC horizon requires reviewing the model's fixed-size outputs,
not just changing `Np`.

## Check Changes Before Comparing Results

```matlab
addpath(pwd, genpath('matlab'), genpath('tools'));
testResults = runtests('tests');
assertSuccess(testResults);

% Explicit 120 s integration check; creates a case folder:
addpath('tests/integration');
run_navigation_smoke_test;
```

```shell
python -m unittest discover -s tests -p "test_*.py" -v
```

The smoke test uses seed 7 and refuses to overwrite its previous case. Archive
that case outside its original path before repeating it, or run a new 120 s
case with a different seed. A short setup check does not validate encounter
safety; changes to guidance need encounter-length runs and relevant comparisons.

Commit the reviewed changes before a research campaign. The runner records
the commit, dirty flag, options and seeds in each output folder. Keep the whole
case folder for later analysis; `results/` is intentionally not versioned.
Update affected tests and documentation with behavioral changes. A single
demo is not a statistical validation; see
[reproducibility limits](model-provenance.md#reproducibility-limits).

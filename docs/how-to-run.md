# How To Run

## Requirements

- MATLAB R2026a (the locally tested release), Simulink and Aerospace Blockset.
- Aerospace Toolbox and its `Ephemeris Data for Aerospace Toolbox` add-on.
- Optimization Toolbox for `fmincon` and Control System Toolbox for
  `unscentedKalmanFilter`.
- Optional Python dependencies in
  [`requirements.txt`](../tools/visualization/requirements.txt) for figures/GIFs.

Download or clone the repository and make its root the MATLAB current folder.
If missing `ephMoon405.mat`, `ephEarthMoonBarycenter405.mat` or `ephSun405.mat`
is reported, install the ephemeris add-on. It is not redistributed here.

## Batch Runs

With `inoas_model` closed:

```matlab
caseDir = run_inoas_case('reactive', 'adaptive');
```

The defaults are the degraded AUX3 input, 6743 s, a 720 s MPC horizon, seed 42
and a 35 s minimum acquisition delay. This is a single deterministic demo,
not the paper's original 50-realization campaign.

For an installation check:

```matlab
caseDir = run_inoas_case('reactive', 'adaptive', 'StopTime', 120, 'Seed', 7);
```

The runner deliberately refuses to overwrite an existing case folder or close
an already loaded model. Keep earlier results and use another seed, or archive
the case folder before repeating the identical configuration.

### Policy and Radius Comparisons

```matlab
run_inoas_case('full',     'adaptive',    'Seed', 42);
run_inoas_case('fixed',    'constant295', 'Seed', 42);
run_inoas_case('reactive', 'constant295', 'Seed', 42);
run_inoas_case('reactive', 'adaptive',    'Seed', 42);
% Additional comparator used in the paper:
run_inoas_case('fixed',    'adaptive',    'Seed', 42);
```

Use identical `Seed`, `StopTime` and `InitialError` for paired comparisons.
The seed controls the actuator and auxiliary random sources, not the recorded
GNSS profile. Sharing noise seeds does not imply identical closed-loop states.

Optional parameters:

| Name | Default | Meaning |
| --- | --- | --- |
| `Seed` | 42 | Integer seed; the six derived Simulink seeds are recorded. |
| `StopTime` | 6743 | Seconds, within the supplied 24-hour input. |
| `AcquisitionTime` | 35 | Minimum delay after **every** entry into ACQUIRING. |
| `InitialError` | AUX3 demo error | Six-component estimate-minus-truth error, ECI m and m/s. |

```matlab
run_inoas_case('reactive', 'adaptive', 'Seed', 43, ...
    'AcquisitionTime', 50, 'InitialError', [10; -5; 3; .1; -.1; .05]);
```

Fixed-Time retains its 96 s ON / 300 s OFF calendar when acquisition time is
varied. Reactive retains 60 s nominal tracking / 300 s nominal OFF. Thus a
sensitivity comparison changes the acquisition delay, not both schedules.
The paper's separate 135 s applied-control startup inhibition stays fixed.

## Outputs

Each case is written under
`results/<policy>_<radius>_seed<seed>_<duration>s/` (with an `_acq<delay>s`
suffix for non-default acquisition delays):

- `configuration.json`: options, seeds, MATLAB version, source commit and dirty flag.
- `simulation.mat`: complete `Simulink.SimulationOutput` and configuration.
- `timeseries.csv`, `control.csv`, `applied_control.csv`, `navigation.csv`, `metrics.csv`.
- `raw_visualization_data.mat`: compact data for the Python renderer.
- `completion.json`: success or the error from an incomplete attempt.

`results/cache/` holds regenerated reference/debris trajectories. All results,
caches and Simulink build files are ignored by Git. The batch runner temporarily
isolates old tuning preferences and restores them afterward; initialization
replaces simulation variables in the MATLAB base workspace.

## Graphical Workflow

| Script | Stop time | Purpose |
| --- | ---: | --- |
| `open_inoas_fast` | 120 s | Installation check. |
| `open_inoas_debris_demo` | 1800 s | Includes the encounter designed at 1500 s. |
| `open_inoas_model` | 6743 s by default | Full default run. |

```matlab
open_inoas_debris_demo
out = sim('inoas_model');
run('matlab/plot_MPC_results.m');
```

All modes retain `Np = 60` and `h = 12 s`. The model has fixed-size MPC outputs;
do not reduce the horizon to speed up the demo without changing its dimensions.
For explicit GUI options, create `inoasRunConfig = inoasPaperConfig(...)` before
initialization. For acquisition-only experiments after initialization, set
`gnss_min_cfg.acquisitionTime = 50` before starting a fresh simulation at zero.
Reinitialization restores the selected run configuration.

## Tests and Notes

```matlab
addpath(pwd, genpath('matlab'), genpath('tools'));
results = runtests('tests');
assertSuccess(results);
run_navigation_smoke_test;
```

The smoke test runs 120 s and checks acquisition, tracking, OFF and exports.
It does not certify encounter safety or reproduce ensemble results. Simulink's
existing inferred-dimension warnings for Ground/Demux are documented; they do
not prevent the locally tested run. Nonlinear MPC and repeated UKF forecasts
make full runs substantially slower than simulated time on some machines.

See [model provenance](model-provenance.md), [data](../data/README.md), and
[visualization workflow](visualization-workflow.md).

# Data Files

This folder contains the active data files used by the MATLAB/Simulink
simulation.

- `full_perturb_POS_s6a_Y24D011_fixed.dat` - default S2 degraded GNSS input,
  including the additional satellite-count and geometry perturbations described
  below. This is the input used by the recent AUX3 S2 figure runs.
- `cov_perturb_POS_s6a_Y24D011_fixed.dat` - previous input, retained unchanged
  for explicit replay of earlier experiments; it is no longer the default.
- `perturb_POS_s6a_Y24D011.dat` - original GNSS perturbation dataset retained
  for traceability and comparison with earlier processing.
- `referenceTrajectory.mat` - nominal J2-propagated reference trajectory used by
  the MPC guidance layer. The main setup script can regenerate this file.
- `debrisTrajectory.mat` - debris encounter trajectory used by the avoidance
  scenario. The main setup script can regenerate this file.

Simulation plots should be regenerated from the selected scenario setup rather
than treated as fixed repository outputs.

## Default S2 Input

The default file is a Sentinel-6A-derived test profile with controlled
perturbations, not an independent flight record of the injected events. It
contains 8640 rows at 10 s spacing, from 0 to 86390 s. The columns are:

```text
SOD LONEST LATEST ALTEST CLKEST GGTO SOL NSVVIS NSV HPE VPE EPE NPE UPE HDOP VDOP PDOP
```

Compared with the legacy `cov_perturb_POS_s6a_Y24D011_fixed.dat`, exactly 52 rows
differ, in the following half-open intervals:

| Modified interval [s] | Changed columns | Effect |
|---|---|---|
| [2000, 2260) | NSVVIS, NSV, HDOP, VDOP, PDOP | Satellite counts reduced to 1-2; DOP values also modified. |
| [3500, 3760) | HDOP, VDOP, PDOP | Geometry indicators modified, with PDOP above 6 only in [3500, 3550). |

All other values are unchanged, including EPE/NPE/UPE and HPE/VPE. In
particular, the [700, 980) event and the later events outside the 6743 s study
arc are inherited from the legacy file. The new file was copied unchanged from
the archived S2 input; this update does not generate new perturbations.

In the 6743 s study arc, the three degradation intervals rejected by the quality
gate are **[700, 980), [2000, 2260), and [3500, 3550) s**. The gate uses finite
SOL, NSV and PDOP values, with SOL >= 0.5, NSV >= 5 and 0 < PDOP <= 6. HPE/VPE
are reference-error metrics, not onboard acceptance criteria. Initial no-fix
placeholders are handled by the loader's initial-quality padding.
These intervals describe input quality; actual updates also depend on receiver
state and measurement epochs.

The five-satellite count threshold is consistent with a GPS/Galileo position
solution estimating three position components and two receiver clock terms.
It is a necessary count condition for that formulation, not a guarantee of
full geometry-matrix rank. The replay supervisor uses NSV (satellites used),
not NSVVIS (satellites visible), together with solution validity and PDOP.
None of the three versioned GNSS files contains an NSV value of exactly four,
so changing this threshold from four to five leaves their quality decisions
unchanged. This equivalence must be checked again for other input profiles.

The file does not contain the auxiliary-sensor fault injections, actuator
noise, receiver policy, or MPC settings. The 10 s data spacing and the model's
existing 3 s GNSS measurement calendar are unchanged by this dataset update.
The MPC reference orbit is generated separately from the GNSS error replay.

`initialize_inoas_simulation.m` selects the file through `gnssCovarianceFile`.
The model's InitFcn loads quality signals from that same selection, falling
back to the default S2 file when no selection exists. The GNSS helper functions
also default to S2 when called without a filename. An explicitly supplied
legacy filename remains supported.

## Regression Check

From the repository root in MATLAB, without running a spacecraft simulation:

```matlab
results = runtests('tests');
assertSuccess(results);
```

# GNSS Data

| File | Role |
| --- | --- |
| `full_perturb_POS_s6a_Y24D011_fixed.dat` | **Default** processed error/quality profile with controlled degradation. |
| `cov_perturb_POS_s6a_Y24D011_fixed.dat` | Earlier processed profile retained for comparison/traceability. |
| `perturb_POS_s6a_Y24D011.dat` | Original supplied perturbation profile retained for traceability. |

The default has 8640 records, 17 columns and a 10 s cadence over 24 hours
(`t = 0 ... 86390 s`). It supplies Sentinel-6A-derived position-error profiles
and quality indicators, not the simulated spacecraft trajectory. The raw PPP
engine, raw observables and its software/version provenance are not bundled.

In the first 6743 s, quality screening rejects [700,980), [2000,2260) and
[3500,3550) s. These intervals are separate from the synthetic auxiliary bias
pulses at [600,620) and [2000,2020) s. Quality is healthy only when Sol, Nsat
and PDOP are finite, Sol >= 0.5, Nsat >= 5 and 0 < PDOP <= 6. HPE/VPE do not
control receiver power or measurement acceptance.

## Observation Approximation

Initialization reads EPE/NPE/UPE and uses `[NPE,EPE,UPE]` in that order as ECI
x/y/z position disturbances. This is **componentwise reuse, not an ENU-to-ECI
coordinate transformation**. Velocity disturbances are `gradient(series,10)`
at the native data spacing. Errors are linearly interpolated between records;
the true plant state is sampled at the 3 s GNSS measurement epochs.

The UKF uses a fixed nominal GNSS covariance with standard deviations 5 m
and 0.1 m/s, not a covariance fitted at each epoch from HPE/VPE. The auxiliary
observations are synthetic ECI positions with a 2000 m standard deviation.
These approximations are inherited from the AUX3 development model, not calibrated
receiver/auxiliary hardware measurements.

## Paths and Generated Files

`inoas_data_file` resolves these versioned files. Both the initializer and the
Simulink `InitFcn` use the selected `gnssCovarianceFile`; neither silently
falls back to the old processed profile in a default run.

Generated `referenceTrajectory.mat` and `debrisTrajectory.mat` now live in
ignored `results/cache/` and are regenerated at initialization. They are no
longer tracked alongside immutable GNSS inputs. See
[How To Run](../docs/how-to-run.md) and [provenance](../docs/model-provenance.md).

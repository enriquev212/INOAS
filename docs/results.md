# Simulation Diagnostics

Run a case using [the run guide](how-to-run.md), then inspect its exported
configuration together with its plots. This page describes diagnostics,
not benchmark values or a precomputed performance campaign.

## Generated Plots

`matlab/plot_MPC_results.m` generates:

- 3D orbital trajectory with reference, truth, estimated state and debris path.
- Position tracking against the nominal reference and estimation-error norms.
- Applied control acceleration with actuator limits.
- Debris separation against the physical keep-out distance and safety radius.
- Dynamic safety radius over the prediction horizon.
- XY, XZ and YZ trajectory projections.
- RTN radial/tangential/normal tracking errors and control effort.
- Cumulative maneuver Delta-v and an MPC diagnostic summary.

For PDF/PNG diagnostics and animations from exported cases, see
[Visualization Workflow](visualization-workflow.md).

## Interpreting a Run

- Check `configuration.json` for receiver policy, radius mode, acquisition
  delay, initial error, seeds, duration and source revision.
- Receiver energy is integrated from OFF/ACQUIRING/TRACKING occupancy,
  not from the GNSS correction-enable flag.
- Commanded and applied Delta-v are distinct; startup inhibition and actuator
  imperfections affect the applied value.
- Generic position-error metrics include startup. Match the evaluation
  window, initial error and seeds before comparing configurations.
- Exported minimum debris separation is sampled, not an exact continuous-time
  closest-approach calculation or a safety certificate.

See [metric definitions](../tools/visualization/README.md#metrics),
[model assumptions](model-architecture.md) and
[reproducibility limits](model-provenance.md#reproducibility-limits).

## Further Development

Useful extensions include physical auxiliary-sensor models, attitude/thrust
coupling, debris-state uncertainty, additional encounter and degradation
profiles, receiver hardware characterization and onboard-computation profiling.
Tests and broader simulation coverage are needed before drawing conclusions
beyond a particular setup.

[Challenge communication material](history.md) belongs to an earlier
configuration and should not be treated as current-run output.

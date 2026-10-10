# Model Provenance

## Active Version

`main` contains the supported CubeSat demonstrator. The Simulink model and
navigation/guidance core come from the team's AUX3 development revision
`721c0eb19e7e5edced4424ad8dbb538ce946125f`. Subsequent maintenance added
configuration options, seeded runs, exports and tests without retuning that
core. The current setup differs from the earlier Challenge configuration.

Record the source revision and run configuration when comparing experiments.
The runner writes the Git commit, dirty flag, MATLAB version, options and
derived seeds into each case folder. A dirty flag means the commit alone
does not identify every local edit.

## Archived Development Branches

Superseded development states are preserved in a maintainer-held Git bundle,
`inoas-public-history-20261011.bundle`. They are historical references, not
alternative supported defaults. The bundle was independently restored and
checked with `git bundle verify` and `git fsck --full`.

Request the archive from the maintainer if an older state is needed. Public
Git history has not been rewritten; retiring branches or tags does not make
previously published history or external copies confidential. The original
[Challenge-materials Release](https://github.com/enriquev212/INOAS/releases/tag/inoas-project-materials-v1)
is retained.

## Public Workflow

- `run_inoas_case` exposes receiver policy, radius mode, seed, acquisition
  delay, duration and optional initial estimation error.
- MATLAB and Simulink random sources are explicitly seeded and recorded.
- Generated trajectories, cases and build caches are ignored by Git.
- Outputs distinguish receiver power from GNSS correction enable and applied
  acceleration from commanded acceleration.
- The historical two-state supervisor is isolated under `tests/legacy/`,
  outside the active MATLAB path.

See [development](development.md) for extension points and checks.

## Reproducibility Limits

A default run is one configured demonstration, not a statistical validation.
The default initial estimate error is inherited from AUX3; use `InitialError`
to select a different one. For paired comparisons, match the noise seed,
initial error, duration and input profile as well as the metric window.

The repository contains processed Sentinel-6A-derived profiles, not the raw
GNSS processing engine or a verified software/version manifest for it.
Raw PPP is not executed inside Simulink. See [data assumptions](../data/README.md).

Synthetic auxiliary observations, one encounter setup and an assumed receiver
power model limit the interpretation of an individual run. Debris-state
uncertainty is omitted. Exact numerical identity across MATLAB/solver versions
and platforms is not asserted, and simulation constraints are not flight
qualification or an unconditional collision-safety guarantee.

MATLAB regression tests, Python exporter/visualization tests and an explicit
short Simulink smoke test are provided. A short check cannot replace
encounter-length tests for changes to guidance. Existing Ground/Demux
inferred-dimension warnings should be reviewed separately from test failures.

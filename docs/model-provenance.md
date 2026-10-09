# Model Provenance

## Source Revisions

The public refresh uses former branch `feat/aux3-on-alberto` at
`721c0eb19e7e5edced4424ad8dbb538ce946125f` as its scientific base. The team
identified this branch as the likely source used for the submitted paper.
That revision is preserved under the `archive/aux3-on-alberto-20261009` tag.
`models/inoas_model.slx` is byte-identical to that revision. The MPC, forecast
and measurement/dynamics core were imported from the same branch.

The previous default and the recently updated paper documentation remain in
Git history at `196af19df1c364af2c4fb94581ebd3645e0bb9fb`. The update preserves
the challenge GIF/poster assets and the reported paper results rather than
replacing them with a new demo. Superseded remote branches are preserved by
archive tags as described below; their history is not discarded.

## Archived Development Branches

`main` is the only active remote branch after the 9 October 2026 cleanup.
The following annotated tags preserve the exact former branch tips, including
their ancestors and historical model files. These are historical references,
not alternative supported defaults.

| Former branch | Archive tag | Commit |
| --- | --- | --- |
| `feat/aux3-on-alberto` | [`archive/aux3-on-alberto-20261009`](https://github.com/enriquev212/INOAS/tree/archive/aux3-on-alberto-20261009) | `721c0eb19e7e5edced4424ad8dbb538ce946125f` |
| `fix/alberto-debris-j2-20260917` | [`archive/alberto-debris-j2-20260917`](https://github.com/enriquev212/INOAS/tree/archive/alberto-debris-j2-20260917) | `1a558b558141e29b7b49de8753de4ecea91cd79f` |
| `fix/mpc-open-points` | [`archive/mpc-open-points-20261009`](https://github.com/enriquev212/INOAS/tree/archive/mpc-open-points-20261009) | `81496d47d396adb0628782da91073a4fbdd4ad20` |
| `recover-domingo` | [`archive/recover-domingo-20261009`](https://github.com/enriquev212/INOAS/tree/archive/recover-domingo-20261009) | `48d567844fc5b21df14c346346707fdbf1b841df` |
| `recover-domingo-altimetro` | [`archive/recover-domingo-altimetro-20261009`](https://github.com/enriquev212/INOAS/tree/archive/recover-domingo-altimetro-20261009) | `cdbb9e5b44eeae0d6348ded838eb6c1b431c6dc7` |

The already merged `docs/final-paper-results` and
`maintenance/github-paper-refresh` branches are also removed; their commits
remain in `main`. The `inoas-project-materials-v1` release tag is unchanged.
Local working copies and uncommitted experiments are not removed or reset.

To inspect an old version without changing the active checkout:

```shell
git fetch origin --tags
git worktree add --detach ../INOAS-historical archive/aux3-on-alberto-20261009
```

Do not use an archive tag as evidence that its full configuration reproduces
the submitted 50-run campaign; the limits below still apply.

## Public-Runner Changes

- Default duration is 6743 s instead of the AUX3 script's 4000 s demo.
- `inoasPaperConfig` and `run_inoas_case` expose receiver policy, radius, seed,
  acquisition delay, duration and optional initial estimation error.
- Reactive's transition algorithm is retained. Full GNSS disables nominal
  power-off but retains acquisition/quality screening; Fixed-Time uses the
  96 s ON / 300 s OFF calendar from the team's earlier comparison driver.
- MATLAB and Simulink random sources are seeded explicitly and recorded.
- Generated reference/debris MAT files move to ignored `results/cache/`.
- Outputs distinguish receiver power from GNSS correction enable and applied
  acceleration from commanded acceleration.
- Obsolete two-state rewiring scripts and their old tests/instructions are
  removed. [`tests/legacy/instrument_decision.m`](../tests/legacy/instrument_decision.m)
  remains only as a legacy regression reference outside the active MATLAB path,
  not the active supervisor.
- Diagnostics/comments are corrected; the GNSS frame approximation, gravity,
  force models, noise assumptions, MPC weights and auxiliary biases are retained.

These changes make the model easier to run and audit; they are not a new
physics calibration or an independently reproduced paper campaign.

## Reproducibility Limits

The original 50-run seed list, randomized initial errors and complete outputs
are not included. Earlier locally identified 24-draw campaigns are not relabeled
as the final campaign. A default run uses the original AUX3 demonstration error
`[1000; -750; 500; .75; -.50; .25]`, in ECI m and m/s, unless `InitialError`
is specified. Reported ensemble navigation errors exclude the common startup
interval; public demo metrics include it unless explicitly named otherwise.

The repository contains processed Sentinel-6A-derived profiles, not the raw
GNSS processing engine or a verified software/version manifest for it. The
24-hour errors and quality indicators are replayed; raw PPP is not run inside
Simulink. See [data assumptions](../data/README.md).

Each new run records its Git commit, dirty flag, MATLAB version and source
options. A dirty flag means the commit alone does not identify every edit;
commit a reviewed configuration before using it for a publication campaign.
Exact numerical identity across MATLAB versions, solver versions and platforms
is not asserted.

The reported paper results are conditional on one encounter geometry, the
specified degradation/bias profile, synthetic auxiliary observations and the
receiver power model. They do not establish flight qualification or a general
collision-probability bound.

## Local Verification (9 October 2026)

MATLAB R2026a Update 4: 31 tests passed for input selection, quality screening,
receiver policies, covariance forecasts and CSV/MAT exports. Four 120 s
Simulink cases covered Full/adaptive, Fixed-Time/constant295,
Reactive/constant295 and Reactive/adaptive. A separate 1800 s
Reactive/adaptive run completed through the design encounter with CSV/MAT
exports. Existing Ground/Demux inferred-dimension warnings remain.

Five Python regression tests passed. PDF/PNG diagnostics and an encounter GIF
were rendered from the new outputs. All 52 local Markdown link targets were
checked. The distributed Simulink blob matches AUX3 (`04052b98...`), and the
challenge GIF matches the previous main (`0f5bcaca...`).

This verification did not rerun a complete 6743 s case or the original 50-run
campaign. It checks the public migration and workflow, not the paper statistics.

### Branch and Legacy Cleanup

After relocating the historical supervisor to `tests/legacy/`, all 32 MATLAB
tests and five Python tests passed. The added test checks that the historical
file is absent from `matlab/` and that both supervisor functions resolve to
their intended locations. The model's embedded wrapper still calls
`inoasMinimalGnssStep`; the Simulink file, MPC, active supervisor and visual
assets are unchanged by this cleanup. Archive tags were verified on GitHub
against the full former branch-tip SHAs before deleting any branch.

# Model Provenance

## Source Revisions

The public refresh uses the AUX3 revision at
`721c0eb19e7e5edced4424ad8dbb538ce946125f` as its scientific base. The team
identified this revision as the likely source used for the submitted paper.
That revision is preserved in the maintainers' non-public Git bundle.
`models/inoas_model.slx` is byte-identical to that revision. The MPC, forecast
and measurement/dynamics core were imported from the same revision.

The previous default and the recently updated paper documentation remain in
Git history at `196af19df1c364af2c4fb94581ebd3645e0bb9fb`. The update preserves
the challenge GIF/poster assets and the reported paper results rather than
replacing them with a new demo. Superseded development branches are preserved
outside the public repository as described below; their history is not discarded.

The [IEEE 2027 paper-model Release](https://github.com/enriquev212/INOAS/releases/tag/v1.0-ieee2027-paper-model)
pins the public runnable model and documentation. It does not identify the
original submitted campaign's exact seeds, initial errors or complete outputs.

## Archived Development Branches

`main` is the only active remote branch. On 11 October 2026, the five historical
development tags were retired from GitHub after creating and independently
restoring a complete Git bundle. The annotated tags, exact former branch tips,
ancestors and historical model files remain in that maintainer-held archive.
These are historical references, not alternative supported defaults.

| Historical revision | Commit |
| --- | --- |
| AUX3 scientific base | `721c0eb19e7e5edced4424ad8dbb538ce946125f` |
| Debris J2 correction | `1a558b558141e29b7b49de8753de4ecea91cd79f` |
| MPC development revision | `81496d47d396adb0628782da91073a4fbdd4ad20` |
| Earlier recovery baseline | `48d567844fc5b21df14c346346707fdbf1b841df` |
| Earlier altitude-channel variant | `cdbb9e5b44eeae0d6348ded838eb6c1b431c6dc7` |

The already merged `docs/final-paper-results` and
`maintenance/github-paper-refresh` branches are also removed; their commits
remain in `main`. The `inoas-project-materials-v1` release tag is unchanged.
Local working copies and uncommitted experiments are not removed or reset.

The archive is named `inoas-public-history-20261011.bundle`. Request it from
the maintainer if an older development state is needed. A separate repository
can be restored without changing the active checkout:

```shell
git init --bare INOAS-history.git
git --git-dir=INOAS-history.git fetch /path/to/inoas-public-history-20261011.bundle "refs/tags/*:refs/tags/*" "refs/remotes/origin/main:refs/heads/main"
git --git-dir=INOAS-history.git symbolic-ref HEAD refs/heads/main
git --git-dir=INOAS-history.git fsck --full
```

The bundle was verified with `git bundle verify`, restored into an independent
bare repository, and checked with `git fsck --full`; all five annotated tag
objects and peeled commit IDs matched. Public Git history was not rewritten,
and existing local tags and working copies were left intact. Retiring public
tags does not remove objects still reachable from `main` or the challenge
Release, nor does it make already published development history confidential.

Do not use an archived revision as evidence that its full configuration
reproduces the submitted 50-run campaign; the limits below still apply.

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

## Paper Documentation and Figures (10 October 2026)

The team supplied the paper architecture and supporting-plane geometry as
vector PDFs. Their unchanged originals and rendered PNG previews are under
[`docs/assets/paper/`](assets/paper/README.md). The current architecture page
uses these figures and explains how the functional diagram maps to the
simulated GNSS replay, synthetic auxiliary channel and UKF estimate.

Superseded challenge poster/architecture assets are isolated under
`docs/assets/history/` and [project history](history.md). The challenge GIF
remains unchanged at its original path and is explicitly historical. The main
README, results and reference list now focus on the submitted paper; the
reported results table also includes its receiver power-on counts.

This documentation update does not change the plant, observation generation,
receiver policies, MPC, default inputs or any numerical result. It does not
resolve the missing original campaign seeds/initial errors described above.

Verification: 92 local Markdown targets and 16 heading anchors resolve;
the five Python visualization tests pass. The supplied PDFs match their
source-file hashes, both PNG previews were visually inspected, and the moved
historical assets and preserved GIF match their previous Git blobs. No MATLAB,
Simulink, Python or dataset source files were changed by this update.

## Project Organization (10 October 2026)

The root now keeps the initializer and two main entry points. The three
convenience launchers moved to `examples/` with repository-relative path
resolution; MATLAB Online instructions moved to `docs/matlab-online.md`.
The README is shorter and the unchanged challenge GIF is prominent again,
with its historical configuration identified. A documentation index and
development guide retain the detail needed to continue the work.

All 35 MATLAB tests and five Python tests pass. Three added launcher tests
use isolated stubs to check durations and paths from outside the repository;
they do not run new Simulink simulations. The active model, dynamics,
receiver algorithms, initializer, batch runner, data and visual assets are
unchanged. This organization update does not reproduce a new paper campaign.

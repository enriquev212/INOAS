# MATLAB Online Quick Trial

Upload/extract the repository and enter the root `INOAS` folder. MATLAB Online
needs the same licensed toolboxes and ephemeris data as the desktop version;
see [requirements](how-to-run.md#requirements).

```matlab
caseDir = run_inoas_case('reactive', 'adaptive', 'StopTime', 120, 'Seed', 7);
```

This checks setup and exports CSV/MAT files into a fresh `results/` subfolder.
For an encounter demo, use `StopTime = 1800`; for the default full run, omit
`StopTime` (6743 s). The 12 s MPC step and 60-step horizon are unchanged in
short tests. No manual block editing or model save is required.

For a 1000 s Reactive/adaptive navigation trial:

```matlab
run('examples/run_navigation_trial.m');
```

Download the **complete case folder**, including configuration, simulation and
completion files, before interpreting figures. The runner does not overwrite
previous results. Paired cases use the same seed and initial error; a new
seed alone does not reconstruct the original 50-realization paper campaign.

Render PDF/PNG figures locally:

```powershell
python -m pip install -r tools/visualization/requirements.txt
python tools/visualization/render_campaign_figures.py --case results/reactive_adaptive_seed7_120s
```

For receiver assumptions and reproducibility limits, see
[architecture](model-architecture.md) and [provenance](model-provenance.md).

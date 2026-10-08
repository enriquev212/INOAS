# Visualization Workflow

Use the reproducible batch runner first, then render separately in Python:

```matlab
caseDir = run_inoas_case('reactive', 'adaptive', 'Seed', 42);
```

```powershell
python -m pip install -r tools/visualization/requirements.txt
python tools/visualization/render_campaign_figures.py --case results/reactive_adaptive_seed42_6743s
python tools/visualization/generate_visualization_assets.py --mat results/reactive_adaptive_seed42_6743s/raw_visualization_data.mat --out results/reactive_adaptive_seed42_6743s/animation
```

Paper-style PDF/PNG diagnostics go into `<case>/figures/`; animation assets
go into the requested `animation/` folder. MATLAB is not needed for re-rendering
an already exported compact MAT/CSV case. The dependencies are NumPy, SciPy,
Matplotlib and Pillow.

The renderer uses actual receiver-state energy when available. A correction
enable flag alone cannot distinguish powered acquisition from sleep. Applied
and commanded Delta-v are different quantities; see
[metric definitions](../tools/visualization/README.md#metrics).

For a manual GUI simulation, keep the initializer workspace and run:

```matlab
out = sim('inoas_model');
export_campaign_csv('results/manual_case');
export_visualization_data('results/manual_case/raw_visualization_data.mat');
```

The runner is preferred because it records seeds/options and adds the required
receiver-state and applied-acceleration logs without saving model edits.
The historical GIF under `docs/assets/` is preserved; new default-run assets
must not be presented as the original 50-run campaign or the challenge-final run.

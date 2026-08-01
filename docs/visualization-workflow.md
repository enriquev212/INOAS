# Visualization Workflow

The repository includes an optional export-and-render workflow for
presentation-style assets. This is useful when the scenario, `StopTime`, or MPC
horizon has changed and the GIF/figures should match the current simulation.

## MATLAB Export

After running Simulink, export a compact MATLAB data file:

```matlab
out = sim("inoas_model");
export_visualization_data
```

The default export is:

```text
results/visualization/inoas_visualization_data.mat
```

## Python Rendering

Install the optional Python dependencies:

```powershell
python -m pip install -r tools\visualization\requirements.txt
```

Then generate the assets from the repository root:

```powershell
python tools\visualization\generate_visualization_assets.py
```

By default, the workflow writes to `results/visualization/`, which is ignored by
Git because these files are generated from the selected simulation setup.

The Python script creates:

- `inoas_orbit_context.png`
- `inoas_debris_distance.png`
- `inoas_control_energy_summary.png`
- `inoas_debris_avoidance.gif`
- `inoas_visualization_summary.txt`

The lower-level tool documentation is available in
[`tools/visualization/README.md`](../tools/visualization/README.md).

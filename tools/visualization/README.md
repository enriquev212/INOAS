# Visualization Tools

These tools regenerate presentation-style figures from a completed INOAS
Simulink simulation.

## Workflow

From MATLAB, run a simulation first:

```matlab
open_inoas_debris_demo
out = sim("inoas_model");
export_visualization_data
```

Then generate the PNG/GIF assets from the repository root:

```powershell
python tools\visualization\generate_visualization_assets.py
```

The default MATLAB export is:

```text
results/visualization/inoas_visualization_data.mat
```

The Python script writes:

- `inoas_orbit_context.png`
- `inoas_debris_distance.png`
- `inoas_control_energy_summary.png`
- `inoas_debris_avoidance.gif`
- `inoas_visualization_summary.txt`

The generated `results/` folder is ignored by Git because these files depend on
the selected scenario, `StopTime`, MPC horizon, and logged Simulink signals.

## Python Requirements

The generator uses:

- `numpy`
- `scipy`
- `matplotlib`
- `Pillow`

Install them with:

```powershell
python -m pip install -r tools\visualization\requirements.txt
```

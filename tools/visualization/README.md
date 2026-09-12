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

## CSV Campaign Workflow for Paper Figures

For the conference paper, prefer the CSV campaign workflow. It separates the
expensive Simulink run from figure rendering:

1. Run the baseline case in MATLAB or MATLAB Online:

```matlab
run_baseline_campaign
```

By default this runs the model to `StopTime = 1000 s` and writes:

```text
results/campaign/baseline/
  raw_visualization_data.mat
  timeseries.csv
  control.csv
  navigation.csv
  metrics.csv
```

Use an explicit output folder or stop time if needed:

```matlab
run_baseline_campaign("results/campaign/baseline_1000s", 1000)
```

2. Download the `results/campaign/baseline/` folder from MATLAB Online.

3. Render paper-style figures with Python:

```powershell
python tools\visualization\render_campaign_figures.py --case results\campaign\baseline
```

The script writes PDF and PNG figures to:

```text
results/campaign/baseline/figures/
```

These figures are intended for LaTeX inclusion, for example:

```latex
\includegraphics[width=\columnwidth]{figures/fig6_collision_margin.pdf}
```

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

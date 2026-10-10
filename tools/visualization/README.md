# Visualization Tools

## Run and Export

From the repository root in MATLAB, with the model closed:

```matlab
caseDir = run_inoas_case('reactive', 'adaptive', 'Seed', 42);
```

The runner writes complete simulation/configuration files, a compact MAT and
five CSVs to `results/reactive_adaptive_seed42_6743s/`.
`run_baseline_campaign` is a no-argument alias for the same default case.
Generated files are ignored by Git and remain local case outputs.

## Python Figures

```powershell
python -m pip install -r tools/visualization/requirements.txt
python tools/visualization/render_campaign_figures.py --case results/reactive_adaptive_seed42_6743s
```

PDF/PNG files are written to `<case>/figures/`. Use PDF for LaTeX inclusion.
These are single-run diagnostic figures, not a statistical validation.
The CSV minimum separation is sampled; it
does not replace a continuous closest-approach analysis.

## Animation

```powershell
python tools/visualization/generate_visualization_assets.py --mat results/reactive_adaptive_seed42_6743s/raw_visualization_data.mat --out results/reactive_adaptive_seed42_6743s/animation
```

This creates orbit, distance, control/energy PNGs and a debris-avoidance GIF.
An encounter-length run is needed for a meaningful encounter animation. The
README's existing challenge GIF remains unchanged and labeled as historical.

## Metrics

- `lambda` is correction enable; `receiver_on` is power and `receiver_mode`
  is OFF/ACQUIRING/TRACKING (0/1/2).
- Energy uses left-held receiver states: 0.025/2.34/1.8 W, divided by 3600 for Wh.
- `control.csv` contains commanded acceleration and commanded Delta-v.
- `applied_control.csv` contains actuator-imperfect acceleration and applied Delta-v.
- `metrics.csv` distinguishes `final_commanded_delta_v_mps` from the applied
  `final_delta_v_mps` (after startup command inhibition and including actuator
  imperfections). The native model inhibits applied control through 135 s.
  Commanded Delta-v is a trapezoidal diagnostic on the MPC sample grid. Missing applied
  logs produce NaN, not a silently relabeled commanded value.
- Position errors in the generic exporter include startup. Match the
  evaluation window explicitly when comparing cases.

For manual simulations, `export_campaign_csv` and `export_visualization_data`
can export the base-workspace `out`. Receiver energy requires the receiver
state log; it is not inferred from lambda.

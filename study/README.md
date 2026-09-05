# Study scripts

Everything that produces a number or a figure for the paper lives here, inside
the repository. Each script locates itself with `mfilename('fullpath')`, so the
repository can sit anywhere.

## Reproducing the results

```matlab
cd study
run_navigation_chain    % duty cycle and navigation law, from the full model
mc_cam                  % one Monte Carlo block; see below for the full campaign
mc_surface              % the (sigma_nav, sigma_object) result surface
test_cam_retarget       % the conjunction, planned and executed end to end
make_figures            % figures 1-5
make_cam_figures        % conjunction geometry figures
make_retarget_figures   % execution and coupling
make_arch_figure        % cost of the two architectures
```

`out/` holds the saved results and `figures/` the PDF and PNG output. Both are
committed, so the analysis scripts run without repeating the campaigns.

## The Monte Carlo campaign

1500 geometries, split into six blocks that run as separate MATLAB processes:

```
for i in 1 2 3 4 5 6; do
  matlab -batch "cd study; MC_REPO='<repo>/study/work/c$i'; \
                 MC_CHUNK=$i; MC_NCHUNK=6; MC_N=250; mc_cam" &
done
```

Two things make the split necessary, and both are easy to get wrong:

- `initialize_inoas_simulation.m` **writes** `data/referenceTrajectory.mat` and
  `data/debrisTrajectory.mat`. Concurrent processes sharing one working copy
  overwrite each other's scenario. `sync_work` creates one copy per block under
  `work/`; **call it after every change to the repository**, or the campaign
  measures stale code.
- `setpref('inoas',...)` is global to the MATLAB installation and the
  initialization script consumes it with `rmpref`: a single-use mailbox. Give
  each process its own `MATLAB_PREFDIR` or they steal each other's
  configuration silently.

`MC_CHUNK` seeds the generator, so a block is reproducible given the same code.

# Examples

All examples use the same active Simulink model and keep its 720 s MPC horizon.
They only select a shorter stop time. Run these commands from the repository
root in MATLAB:

| Example | Duration | Purpose |
| --- | ---: | --- |
| [open_inoas_fast.m](open_inoas_fast.m) | 120 s | Open a short setup check |
| [open_inoas_debris_demo.m](open_inoas_debris_demo.m) | 1800 s | Open a run through the encounter |
| [run_navigation_trial.m](run_navigation_trial.m) | 1000 s | Batch run through the first GNSS rejection, with exports |

```matlab
% Interactive encounter demo; opens the model without starting simulation:
run('examples/open_inoas_debris_demo.m');
out = sim('inoas_model');

% Batch alternative: close the model first, preserving any unsaved edits:
% caseDir = run_inoas_case('reactive', 'adaptive', 'StopTime', 1800);
```

The graphical examples retain any existing `inoasRunConfig` configuration;
they do not reset every option to its default. Start in a fresh MATLAB session
for the default Reactive/adaptive setup. The batch runner selects its options
explicitly and records them.

Use [run_inoas_case](../run_inoas_case.m) for new configurations rather than
duplicating a launcher. Full instructions: [How To Run](../docs/how-to-run.md).

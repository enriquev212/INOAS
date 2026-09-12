function outputDir = run_navigation_trial(mode)
%RUN_NAVIGATION_TRIAL Run the versioned navigation trial in MATLAB Online.
% From the INOAS folder: run_navigation_trial
% Optional scheduled-GNSS forecast: run_navigation_trial("navigation_scheduled")
if nargin < 1
    mode = "navigation_aux_only";
end
mode = string(validatestring(mode, ...
    {'navigation_aux_only','navigation_scheduled'}));
repoRoot = fileparts(mfilename('fullpath'));
addpath(repoRoot);
addpath(fullfile(repoRoot,'matlab'));
addpath(genpath(fullfile(repoRoot,'tools')));
stamp = string(datetime('now','Format','yyyyMMdd_HHmmss'));
outputDir = fullfile(repoRoot,'results','campaign',mode + "_" + stamp);
setpref('inoas','mpcTuneConfig',struct('covariancePredictionModeMpc',mode));
outputDir = run_baseline_campaign(outputDir,1000);
end

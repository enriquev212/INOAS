function outputDir = run_baseline_campaign(outputDir, stopTime)
%RUN_BASELINE_CAMPAIGN Run the baseline INOAS case and export CSV results.
%
% This wrapper is meant for MATLAB Online or a fresh MATLAB session:
%
%   run_baseline_campaign
%
% Optional arguments:
%
%   run_baseline_campaign("results/campaign/baseline_1000s", 1000)
%
% The function writes CSV files under outputDir and also keeps the compact MAT
% export used by the existing visualization workflow.

repoRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));

if nargin < 1 || isempty(outputDir)
    outputDir = fullfile(repoRoot, "results", "campaign", "baseline");
end

if nargin < 2 || isempty(stopTime)
    stopTime = 1000;
end

cd(repoRoot);
addpath(repoRoot);
addpath(genpath(fullfile(repoRoot, "matlab")));
addpath(genpath(fullfile(repoRoot, "tools")));
addpath(fullfile(repoRoot, "models"));

modelName = "inoas_model";
modelPath = fullfile(repoRoot, "models", modelName + ".slx");
initScript = fullfile(repoRoot, "initialize_inoas_simulation.m");

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

assignin("base", "repoRoot", repoRoot);
assignin("base", "modelName", modelName);
assignin("base", "simulationStopTime", stopTime);
assignin("base", "inoasCampaignInitScript", initScript);
setpref("inoas", "skipBatchClear", true);
evalin("base", "run(inoasCampaignInitScript); clear inoasCampaignInitScript");

load_system(modelPath);
set_param(modelName, "StopTime", num2str(stopTime));

fprintf("\nRunning INOAS baseline campaign:\n");
fprintf("  model    = %s\n", modelName);
fprintf("  StopTime = %.3f s\n", stopTime);
fprintf("  output   = %s\n\n", outputDir);

assignin("base", "inoasCampaignModelName", modelName);
evalin("base", "out = sim(inoasCampaignModelName); clear inoasCampaignModelName");

if ~isfolder(outputDir)
    mkdir(outputDir);
end

export_visualization_data(fullfile(outputDir, "raw_visualization_data.mat"));
export_campaign_csv(outputDir);
export_navigation_prediction_csv(outputDir, evalin('base','out'));

fprintf("\nBaseline campaign complete.\n");
fprintf("CSV and MAT files are available at:\n%s\n", outputDir);

end

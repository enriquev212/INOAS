%OPEN_INOAS_MODEL Initialize the workspace and open the INOAS Simulink model.

projectRoot = fileparts(mfilename('fullpath'));
modelName = "inoas_model";
modelFile = char(modelName + ".slx");
modelPath = fullfile(projectRoot, "models", modelName + ".slx");

pathEntries = string(strsplit(path, pathsep));
for k = 1:numel(pathEntries)
    pathEntry = char(pathEntries(k));
    if isempty(pathEntry)
        continue
    end

    isCurrentProjectPath = startsWith(lower(pathEntry), lower(projectRoot));
    isInoasProjectPath = isfile(fullfile(pathEntry, 'initialize_inoas_simulation.m')) || ...
        isfile(fullfile(pathEntry, 'MPC_INOAS.m')) || ...
        isfile(fullfile(pathEntry, modelFile));

    if isInoasProjectPath && ~isCurrentProjectPath
        rmpath(pathEntry);
    end
end

projectPathEntries = string(strsplit(genpath(projectRoot), pathsep));
projectPathEntries(projectPathEntries == "") = [];
currentPathEntries = string(strsplit(path, pathsep));
projectPathEntries = projectPathEntries(ismember(projectPathEntries, currentPathEntries));

if ~isempty(projectPathEntries)
    rmpath(strjoin(projectPathEntries, pathsep));
end

cd(projectRoot);
addpath(projectRoot);
addpath(genpath(fullfile(projectRoot, 'matlab')));
addpath(fullfile(projectRoot, 'models'));

check_inoas_requirements();

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

run(fullfile(projectRoot, 'initialize_inoas_simulation.m'));

projectRoot = fileparts(mfilename('fullpath'));
modelName = "inoas_model";
modelPath = fullfile(projectRoot, "models", modelName + ".slx");

load_system(modelPath);
open_system(modelName, "window");

try
    set_param(modelName, 'ZoomFactor', 'fit');
catch
    % Some MATLAB/Simulink releases do not expose this view setting.
end

fprintf('\nINOAS model opened: %s\n', modelName);

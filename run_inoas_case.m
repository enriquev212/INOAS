function outputDir = run_inoas_case(receiverPolicy, radiusMode, varargin)
%RUN_INOAS_CASE Run the AUX3 paper model and export a fresh, reproducible case.
% run_inoas_case('reactive', 'adaptive', 'Seed', 42, 'StopTime', 6743)
if nargin < 1, receiverPolicy = 'reactive'; end
if nargin < 2, radiusMode = 'adaptive'; end
root = fileparts(mfilename('fullpath'));
previousPath = path;
previousDir = pwd;
cleanup = onCleanup(@() restoreContext(previousPath, previousDir)); %#ok<NASGU>
cd(root);
addpath(root, fullfile(root, 'models'), genpath(fullfile(root, 'matlab')), genpath(fullfile(root, 'tools')));
cfg = inoasPaperConfig('ReceiverPolicy', receiverPolicy, 'RadiusMode', radiusMode, varargin{:});
outputDir = fullfile(root, 'results', sprintf('%s_%s_seed%d_%gs', ...
    cfg.ReceiverPolicy, cfg.RadiusMode, cfg.Seed, cfg.StopTime));
if cfg.AcquisitionTime ~= 35
    outputDir = sprintf('%s_acq%gs', outputDir, cfg.AcquisitionTime);
end
if isfolder(outputDir)
    error('INOAS:OutputExists', 'This case already exists: %s. Preserve it or choose a new seed.', outputDir);
end
if bdIsLoaded('inoas_model')
    error('INOAS:ModelAlreadyLoaded', 'Close inoas_model before using the batch runner; unsaved models are not closed automatically.');
end
prefNames = {'mpcTuneConfig', 'debrisConfig', 'skipBatchClear'};
savedPrefs = cell(size(prefNames));
present = false(size(prefNames));
for k = 1:numel(prefNames)
    present(k) = ispref('inoas', prefNames{k});
    if present(k)
        savedPrefs{k} = getpref('inoas', prefNames{k});
        rmpref('inoas', prefNames{k});
    end
end
prefCleanup = onCleanup(@() restorePreferences(prefNames, present, savedPrefs)); %#ok<NASGU>
assignin('base', 'inoasRunConfig', cfg);
assignin('base', 'simulationStopTime', cfg.StopTime);
assignin('base', 'modelName', 'inoas_model');
evalin('base', sprintf('run(''%s'')', strrep(fullfile(root, 'initialize_inoas_simulation.m'), '''', '''''')));
assignin('base', 'mpcQuiet', true);
load_system(fullfile(root, 'models', 'inoas_model.slx'));
modelCleanup = onCleanup(@() close_system('inoas_model', 0)); %#ok<NASGU>
set_param('inoas_model', 'StopTime', num2str(cfg.StopTime), 'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on');
stream = RandStream('mt19937ar', 'Seed', cfg.Seed);
seeds = randi(stream, 2^31-1, 6, 1);
noiseBlocks = {'Actuators imperfections1/Altimetry Noise1', ...
    'Actuators imperfections1/Altimetry Noise2', 'Actuators imperfections1/Altimetry Noise3', ...
    'Kalman Filter/Sensor_Simulation_Model/Magnetometer Noise'};
for k = 1:4
    if k < 4, seed = seeds(k); else, seed = seeds(4:6); end
    set_param(['inoas_model/' noiseBlocks{k}], 'seed', mat2str(seed));
end
cfg.NoiseSeeds = seeds;
signals = {'Spacecraft Dynamics', 1, 'truth_position_eci'; ...
    'Kalman Filter', 1, 'Estimated_Pos_x'; 'MPC', 1, 'u_MPC'; ...
    'Sum2', 1, 'applied_acceleration_eci'; ...
    'Instrument Decision/Instrument Decision FSM', 1, 'lambda'; ...
    'Instrument Decision/Instrument Decision FSM', 2, 'receiver_on'; ...
    'Instrument Decision/Instrument Decision FSM', 3, 'receiver_mode'; ...
    'Instrument Decision/Aux score delay', 1, 'pseudo_NIS'};
for k = 1:size(signals, 1)
    ports = get_param(['inoas_model/' signals{k, 1}], 'PortHandles');
    set_param(ports.Outport(signals{k, 2}), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', signals{k, 3}, ...
        'DataLoggingLimitDataPoints', 'off', 'DataLoggingDecimateData', 'off');
end
mkdir(outputDir);
[~, revision] = system('git rev-parse HEAD');
cfg.RepositoryCommit = strtrim(revision);
[~, workingChanges] = system('git status --porcelain');
cfg.RepositoryDirty = ~isempty(strtrim(workingChanges));
cfg.ScientificBaseCommit = '721c0eb19e7e5edced4424ad8dbb538ce946125f';
cfg.MatlabVersion = version;
cfg.AcquisitionToTrackingPowerRatio = 1.3;
cfg.TrackingPowerW = 1.8;
cfg.OffPowerW = 0.025;
cfg.EffectiveInitialError = evalin('base', 'kalman_initial_error');
cfg.AppliedControlStartTimeS = 135;
writeJson(fullfile(outputDir, 'configuration.json'), cfg);
try
    out = sim('inoas_model');
    assert(abs(out.tout(end)-cfg.StopTime) < 1e-8, 'Run ended before its requested duration.');
    assignin('base', 'out', out);
    save(fullfile(outputDir, 'simulation.mat'), 'out', 'cfg', '-v7.3');
    export_campaign_csv(outputDir);
    export_visualization_data(fullfile(outputDir, 'raw_visualization_data.mat'));
    writeJson(fullfile(outputDir, 'completion.json'), struct('completed', true, 'finalTime', out.tout(end)));
catch ME
    writeJson(fullfile(outputDir, 'completion.json'), ...
        struct('completed', false, 'message', getReport(ME, 'extended', 'hyperlinks', 'off')));
    rethrow(ME);
end
fprintf('Fresh case exported to %s\n', outputDir);
end

function restoreContext(previousPath, previousDir)
path(previousPath);
cd(previousDir);
end

function restorePreferences(names, present, values)
for k = 1:numel(names)
    if present(k), setpref('inoas', names{k}, values{k}); end
end
end

function writeJson(filename, value)
fid = fopen(filename, 'w');
assert(fid >= 0, 'Cannot write %s.', filename);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', jsonencode(value, 'PrettyPrint', true));
end

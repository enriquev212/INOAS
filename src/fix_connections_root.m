%% fix_connections_root.m
% =========================================================================
% INOAS - ROOT LEVEL missing connections ONLY
% Model: MPCcontrolledSpacecraft_plant_gnss_with_predictor
%
% Run this script INDEPENDENTLY from fix_connections_inside_KF.m
%
% Connections added:
%   [1] Spacecraft Dynamics/out:1 --> KALMAN FILTER/in:1
%       Position truth was absent from the KF sensor simulation input.
%
%   [2] Spacecraft Dynamics/out:1 --> Enabled\nSubsystem1/in:1
%       Position truth was absent from the GNSS noise block input.
%
%   [3] Enabled\nSubsystem1/out:3 --> KALMAN FILTER/in:4
%       GNSS Cov_matrix was computed but never routed anywhere.
%       NOTE: Run fix_connections_inside_KF.m FIRST so that port 4
%             exists on the KALMAN FILTER subsystem boundary before
%             this line is added.
%
%   [4] From\nWorkspace/out:1   --> ToWS_trace_x  (replaces Terminator4)
%   [5] From\nWorkspace1/out:1  --> ToWS_trace_y  (replaces Terminator5)
%   [6] From\nWorkspace2/out:1  --> ToWS_trace_z  (replaces Terminator6)
%       Heritage GNSS reference signals were silently terminated.
%       After fixing, workspace variables trace_replay_x/y/z are logged.
%
% =========================================================================

modelName = 'MPCcontrolledSpacecraft_plant_gnss_with_predictor';

%% -------------------------------------------------------------------------
%  0. Load model
%% -------------------------------------------------------------------------
fprintf('============================================================\n');
fprintf(' fix_connections_root.m  (ROOT LEVEL only)\n');
fprintf('============================================================\n\n');

if ~bdIsLoaded(modelName)
    load_system(modelName);
    fprintf('[LOAD] Model loaded.\n');
else
    fprintf('[LOAD] Model already loaded.\n');
end

if strcmp(get_param(modelName, 'SimulationMode'), 'accelerator')
    set_param(modelName, 'SimulationMode', 'normal');
    fprintf('[INFO] Simulation mode set to Normal for editing.\n');
end

relPath = @(fullPath) strrep(fullPath, [modelName '/'], '');

%% -------------------------------------------------------------------------
%  1. Locate required blocks at ROOT level
%% -------------------------------------------------------------------------
fprintf('\n--- Locating root-level blocks ---\n');

% Spacecraft Dynamics
tmp = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'SpacecraftDynamics');
assert(~isempty(tmp), 'ERROR: Spacecraft Dynamics not found.');
sdRel = relPath(tmp{1});
fprintf('  [OK] Spacecraft Dynamics   : "%s"\n', sdRel);

% KALMAN FILTER
tmp = find_system(modelName, 'SearchDepth', 1, 'Name', 'KALMAN FILTER');
assert(~isempty(tmp), 'ERROR: KALMAN FILTER not found.');
kfRel = relPath(tmp{1});
fprintf('  [OK] KALMAN FILTER         : "%s"\n', kfRel);

% Enabled Subsystem (GNSS) - identified by internal EnablePort
gnssPath = '';
allSubs = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'SubSystem');
for k = 1:numel(allSubs)
    if ~isempty(find_system(allSubs{k}, 'SearchDepth', 1, 'BlockType', 'EnablePort'))
        gnssPath = allSubs{k};
        break;
    end
end
assert(~isempty(gnssPath), 'ERROR: Enabled Subsystem (GNSS) not found.');
gnssRel = relPath(gnssPath);
fprintf('  [OK] GNSS Subsystem        : "%s"\n', strrep(gnssRel, newline, '\n'));

% FromWorkspace blocks
fwBlocks = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'FromWorkspace');
fprintf('  [OK] FromWorkspace blocks  : %d found\n', numel(fwBlocks));

fprintf('\n--- Applying root-level connections ---\n\n');

%% -------------------------------------------------------------------------
%  CONNECTION [1]
%  Spacecraft Dynamics out:1 --> KALMAN FILTER in:1  (Truth_Position_ICRF)
%% -------------------------------------------------------------------------
fprintf('[1/6] Spacecraft Dynamics/1 --> KALMAN FILTER/1\n');
fprintf('      (position truth for sensor simulation inside KF)\n');
try
    add_line(modelName, [sdRel '/1'], [kfRel '/1'], 'autorouting', 'on');
    fprintf('      [OK] Connected\n\n');
catch ME
    fprintf('      [SKIP] %s\n\n', ME.message);
end

%% -------------------------------------------------------------------------
%  CONNECTION [2]
%  Spacecraft Dynamics out:1 --> Enabled\nSubsystem1 in:1  (true_position)
%% -------------------------------------------------------------------------
fprintf('[2/6] Spacecraft Dynamics/1 --> GNSS Subsystem/1\n');
fprintf('      (position truth for GNSS noise addition)\n');
try
    add_line(modelName, [sdRel '/1'], [gnssRel '/1'], 'autorouting', 'on');
    fprintf('      [OK] Connected\n\n');
catch ME
    fprintf('      [SKIP] %s\n\n', ME.message);
end

%% -------------------------------------------------------------------------
%  CONNECTION [3]
%  Enabled\nSubsystem1 out:3 (Cov_matrix) --> KALMAN FILTER in:4
%
%  PREREQUISITE: fix_connections_inside_KF.m must have been run first,
%  so that port 4 exists on the KALMAN FILTER subsystem boundary.
%% -------------------------------------------------------------------------
fprintf('[3/6] GNSS Subsystem/3 --> KALMAN FILTER/4\n');
fprintf('      (GNSS Cov_matrix to UKF R input — requires port 4 to exist)\n');
fprintf('      PREREQUISITE: run fix_connections_inside_KF.m first!\n');
try
    add_line(modelName, [gnssRel '/3'], [kfRel '/4'], 'autorouting', 'on');
    fprintf('      [OK] Connected\n\n');
catch ME
    fprintf('      [SKIP] %s\n', ME.message);
    if contains(ME.message, 'port')
        fprintf('      [HINT] Port 4 on KALMAN FILTER does not exist yet.\n');
        fprintf('             Run fix_connections_inside_KF.m first, then re-run this.\n');
    end
    fprintf('\n');
end

%% -------------------------------------------------------------------------
%  CONNECTIONS [4] [5] [6]
%  From\nWorkspace  out:1 --> ToWS_trace_x  (replaces Terminator4)
%  From\nWorkspace1 out:1 --> ToWS_trace_y  (replaces Terminator5)
%  From\nWorkspace2 out:1 --> ToWS_trace_z  (replaces Terminator6)
%% -------------------------------------------------------------------------
logVarNames  = {'trace_replay_x', 'trace_replay_y', 'trace_replay_z'};
twBlockNames = {'ToWS_trace_x',   'ToWS_trace_y',   'ToWS_trace_z'};

for i = 1:numel(fwBlocks)
    fwRel = relPath(fwBlocks{i});
    fprintf('[%d/6] %s --> %s\n', 3+i, ...
        strrep(fwRel, newline, '\n'), twBlockNames{i});

    % Delete existing line to Terminator
    try
        ph    = get_param(fwBlocks{i}, 'PortHandles');
        lineH = get_param(ph.Outport(1), 'Line');
        if lineH ~= -1
            dstPortH  = get_param(lineH, 'DstPortHandle');
            dstName   = get_param(get_param(dstPortH, 'Parent'), 'Name');
            if contains(lower(dstName), 'terminator')
                delete_line(lineH);
                fprintf('      [OK] Removed Terminator "%s"\n', dstName);
            else
                fprintf('      [INFO] Output already connected to "%s"\n', dstName);
            end
        end
    catch ME
        fprintf('      [WARN] Could not remove existing line: %s\n', ME.message);
    end

    % Add To Workspace block
    twFullPath = [modelName '/' twBlockNames{i}];
    try
        if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', twBlockNames{i}))
            add_block('simulink/Sinks/To Workspace', twFullPath, ...
                'VariableName',  logVarNames{i}, ...
                'SaveFormat',    'Array', ...
                'MaxDataPoints', 'inf', ...
                'Position', [700, 60 + (i-1)*80, 780, 80 + (i-1)*80]);
            fprintf('      [OK] Added To Workspace (var: %s)\n', logVarNames{i});
        else
            fprintf('      [SKIP] To Workspace block already exists\n');
        end
    catch ME
        fprintf('      [WARN] %s\n', ME.message);
    end

    % Connect
    try
        add_line(modelName, [fwRel '/1'], [twBlockNames{i} '/1'], 'autorouting', 'on');
        fprintf('      [OK] Connected\n\n');
    catch ME
        fprintf('      [SKIP] %s\n\n', ME.message);
    end
end

%% -------------------------------------------------------------------------
%  Save
%% -------------------------------------------------------------------------
fprintf('--- Saving model ---\n');
try
    save_system(modelName);
    fprintf('[OK] Saved: %s.slx\n', modelName);
catch ME
    fprintf('[ERROR] %s\n', ME.message);
end

fprintf('\n============================================================\n');
fprintf(' ROOT connections done.\n');
fprintf(' Check Data Inspector for: trace_replay_x / y / z\n');
fprintf('============================================================\n');

%% fix_missing_connections.m
% =========================================================================
% INOAS Simulink Model - Missing Connection Repair Script
% Model: MPCcontrolledSpacecraft_plant_gnss_with_predictor
%
% Connections added by this script:
%   [1] Spacecraft Dynamics/out:1 --> KALMAN FILTER/in:1
%       (position truth was never reaching the KF sensor simulation)
%
%   [2] Spacecraft Dynamics/out:1 --> Enabled\nSubsystem1/in:1
%       (position truth was never reaching the GNSS noise block)
%
%   [3a] Enable UKF HasMeasurementNoiseCovariancePort = on
%   [3b] Add new Inport 'GNSS_Cov_R' (port 4) inside KALMAN FILTER
%   [3c] GNSS_Cov_R/1 --> Unscented Kalman Filter_/in:3  (inside KF)
%   [3d] Enabled\nSubsystem1/out:3 --> KALMAN FILTER/in:4 (root level)
%       (GNSS Cov_matrix was computed but never used anywhere)
%
%   [4] From\nWorkspace/out:1   --> ToWS_trace_x/in:1  (replaces Terminator4)
%   [5] From\nWorkspace1/out:1  --> ToWS_trace_y/in:1  (replaces Terminator5)
%   [6] From\nWorkspace2/out:1  --> ToWS_trace_z/in:1  (replaces Terminator6)
%       (heritage GNSS reference signals were silently terminated)
%
% IMPORTANT: Run this script ONCE on the original model.
%            Running it twice may produce "already connected" warnings,
%            which are caught and skipped safely.
%
% Requirements: MATLAB R2021a+, Simulink, Control System Toolbox (for UKF)
% =========================================================================

modelName = 'MPCcontrolledSpacecraft_plant_gnss_with_predictor';

%% -------------------------------------------------------------------------
%  0. Load the model
%% -------------------------------------------------------------------------
fprintf('============================================================\n');
fprintf(' INOAS - Missing Connection Repair\n');
fprintf('============================================================\n\n');

if ~bdIsLoaded(modelName)
    fprintf('[LOAD] Loading model: %s ...\n', modelName);
    load_system(modelName);
else
    fprintf('[LOAD] Model already loaded: %s\n', modelName);
end

% Switch to normal mode if in accelerator (needed for structural edits)
if strcmp(get_param(modelName, 'SimulationMode'), 'accelerator')
    set_param(modelName, 'SimulationMode', 'normal');
    fprintf('[INFO] Simulation mode set to Normal for editing.\n');
end

%% -------------------------------------------------------------------------
%  Helper: relative path (block name only, strips "ModelName/")
%% -------------------------------------------------------------------------
relPath = @(fullPath) strrep(fullPath, [modelName '/'], '');

%% -------------------------------------------------------------------------
%  1. Locate all required blocks
%% -------------------------------------------------------------------------
fprintf('\n--- Locating blocks ---\n');

% --- Spacecraft Dynamics ---
tmp = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'SpacecraftDynamics');
assert(~isempty(tmp), 'ERROR: Spacecraft Dynamics block not found.');
sdRel = relPath(tmp{1});
fprintf('  [OK] Spacecraft Dynamics   : "%s"\n', sdRel);

% --- KALMAN FILTER subsystem ---
tmp = find_system(modelName, 'SearchDepth', 1, 'Name', 'KALMAN FILTER');
assert(~isempty(tmp), 'ERROR: KALMAN FILTER subsystem not found.');
kfPath = tmp{1};
kfRel  = relPath(kfPath);
fprintf('  [OK] KALMAN FILTER         : "%s"\n', kfRel);

% --- Enabled Subsystem (GNSS) ---
% Identified by the EnablePort block inside it. Block name contains literal
% newline: 'Enabled\nSubsystem1' - find_system handles this correctly.
gnssPath = '';
allSubs = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'SubSystem');
for k = 1:numel(allSubs)
    ePorts = find_system(allSubs{k}, 'SearchDepth', 1, 'BlockType', 'EnablePort');
    if ~isempty(ePorts)
        gnssPath = allSubs{k};
        break;
    end
end
assert(~isempty(gnssPath), 'ERROR: Enabled Subsystem (GNSS) not found.');
gnssRel = relPath(gnssPath);
fprintf('  [OK] GNSS Enabled Subsystem : "%s"\n', strrep(gnssRel, newline, '\n'));

% --- UKF block inside KALMAN FILTER ---
ukfCandidates = find_system(kfPath, 'SearchDepth', 1, 'RegExp', 'on', ...
                            'Name', 'Unscented.*Kalman');
assert(~isempty(ukfCandidates), 'ERROR: UKF block not found inside KALMAN FILTER.');
ukfPath    = ukfCandidates{1};
ukfRelToKF = strrep(ukfPath, [kfPath '/'], '');
fprintf('  [OK] UKF (inside KF)        : "%s"\n', ukfRelToKF);

% --- FromWorkspace blocks (3 expected) ---
fwBlocks = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'FromWorkspace');
fprintf('  [OK] FromWorkspace blocks   : %d found\n', numel(fwBlocks));

fprintf('\n--- Applying connections ---\n\n');

%% -------------------------------------------------------------------------
%  CONNECTION [1]
%  Spacecraft Dynamics out:1 --> KALMAN FILTER in:1  (Truth_Position_ICRF)
%
%  WHY: KF sensor simulation (Sensor_Simulation_Model) needs the true
%       spacecraft position to generate synthetic measurements.
%       This wire was completely absent from the root level.
%% -------------------------------------------------------------------------
fprintf('[1/6] Spacecraft Dynamics/1 --> KALMAN FILTER/1  (position truth)\n');
try
    add_line(modelName, ...
        [sdRel '/1'], ...
        [kfRel '/1'], ...
        'autorouting', 'on');
    fprintf('      [OK] Connected\n\n');
catch ME
    fprintf('      [SKIP] %s\n\n', ME.message);
end

%% -------------------------------------------------------------------------
%  CONNECTION [2]
%  Spacecraft Dynamics out:1 --> Enabled\nSubsystem1 in:1  (true_position)
%
%  WHY: The GNSS noise block adds realistic measurement noise to the true
%       position. Without this, the GNSS 'position' output has no source.
%% -------------------------------------------------------------------------
fprintf('[2/6] Spacecraft Dynamics/1 --> GNSS Subsystem/1  (position for GNSS noise)\n');
try
    add_line(modelName, ...
        [sdRel '/1'], ...
        [gnssRel '/1'], ...
        'autorouting', 'on');
    fprintf('      [OK] Connected\n\n');
catch ME
    fprintf('      [SKIP] %s\n\n', ME.message);
end

%% -------------------------------------------------------------------------
%  CONNECTION [3]  (4-step process)
%  Enabled\nSubsystem1 out:3 (Cov_matrix) --> KALMAN FILTER in:4 --> UKF R
%
%  WHY: The GNSS measurement noise covariance (R) was computed inside the
%       GNSS subsystem but never passed to the UKF. Feeding it dynamically
%       allows the UKF to tighten/loosen its measurement trust as GNSS
%       geometry changes, which is essential for the duty-cycling logic.
%
%  Step 3a: Enable UKF measurement noise covariance input port
%  Step 3b: Add a new Inport (port 4) inside KALMAN FILTER
%  Step 3c: Wire new Inport --> UKF port 3 (R) inside KALMAN FILTER
%  Step 3d: Wire GNSS out:3 --> new KALMAN FILTER in:4 at root level
%% -------------------------------------------------------------------------
fprintf('[3/6] GNSS Cov_matrix --> KALMAN FILTER in:4 --> UKF R  (4 sub-steps)\n');

% Step 3a: Enable R port on UKF
fprintf('      [3a] Enabling UKF HasMeasurementNoiseCovariancePort ...\n');
try
    currentVal = get_param(ukfPath, 'HasMeasurementNoiseCovariancePort');
    if strcmp(currentVal, 'on')
        fprintf('           [SKIP] Already enabled\n');
    else
        set_param(ukfPath, 'HasMeasurementNoiseCovariancePort', 'on');
        fprintf('           [OK]   Enabled (UKF now has 3 input ports)\n');
    end
catch ME
    fprintf('           [WARN] %s\n', ME.message);
    fprintf('           [INFO] The UKF may use a different parameter name.\n');
    fprintf('                  Manually set HasMeasurementNoiseCovariancePort=on\n');
    fprintf('                  in the UKF block dialog and re-run steps 3c-3d.\n');
end

% Step 3b: Add Inport inside KALMAN FILTER
newInportName = 'GNSS_Cov_R';
newInportFull = [kfPath '/' newInportName];
fprintf('      [3b] Adding Inport "%s" (port 4) inside KALMAN FILTER ...\n', newInportName);
try
    existing = find_system(kfPath, 'SearchDepth', 1, 'Name', newInportName);
    if ~isempty(existing)
        fprintf('           [SKIP] Inport already exists\n');
    else
        add_block('simulink/Sources/In1', newInportFull, ...
            'Port', '4', ...
            'Position', [125, 250, 155, 270]);  % below existing inports
        fprintf('           [OK]   Inport added at port 4\n');
    end
catch ME
    fprintf('           [WARN] %s\n', ME.message);
end

% Step 3c: Wire new Inport --> UKF in:3 (R) inside KALMAN FILTER
fprintf('      [3c] Connecting GNSS_Cov_R/1 --> UKF/3 inside KALMAN FILTER ...\n');
try
    add_line(kfPath, ...
        [newInportName '/1'], ...
        [ukfRelToKF   '/3'], ...
        'autorouting', 'on');
    fprintf('           [OK]   Connected inside KALMAN FILTER\n');
catch ME
    fprintf('           [SKIP] %s\n', ME.message);
end

% Step 3d: Wire GNSS out:3 --> KALMAN FILTER in:4 at root level
fprintf('      [3d] Connecting GNSS/3 --> KALMAN FILTER/4 at root level ...\n');
try
    add_line(modelName, ...
        [gnssRel '/3'], ...
        [kfRel   '/4'], ...
        'autorouting', 'on');
    fprintf('           [OK]   Connected at root level\n\n');
catch ME
    fprintf('           [SKIP] %s\n\n', ME.message);
end

%% -------------------------------------------------------------------------
%  CONNECTIONS [4] [5] [6]
%  From\nWorkspace  out:1 --> ToWS_trace_x  (replaces Terminator4)
%  From\nWorkspace1 out:1 --> ToWS_trace_y  (replaces Terminator5)
%  From\nWorkspace2 out:1 --> ToWS_trace_z  (replaces Terminator6)
%
%  WHY: The 3 heritage GNSS reference signals (used for TRACE duty-cycle
%       replay) were silently killed by Terminator blocks. This makes it
%       impossible to validate the scheduling logic from Section 8.2 of the
%       report. Replacing with To Workspace blocks enables Data Inspector
%       logging and comparison against estimated state.
%
%  NOTE: Variable names in workspace will be:
%        trace_replay_x, trace_replay_y, trace_replay_z
%% -------------------------------------------------------------------------
logVarNames = {'trace_replay_x', 'trace_replay_y', 'trace_replay_z'};
twBlockNames = {'ToWS_trace_x',  'ToWS_trace_y',  'ToWS_trace_z'};
connIdx = 4;

for i = 1:numel(fwBlocks)
    fwRel = relPath(fwBlocks{i});
    fprintf('[%d/6] %s --> %s\n', connIdx, ...
        strrep(fwRel, newline, '\n'), twBlockNames{i});
    
    % --- Delete existing line to Terminator ---
    try
        ph      = get_param(fwBlocks{i}, 'PortHandles');
        lineH   = get_param(ph.Outport(1), 'Line');
        if lineH ~= -1
            dstPortH  = get_param(lineH, 'DstPortHandle');
            dstParent = get_param(dstPortH, 'Parent');
            dstName   = get_param(dstParent, 'Name');
            if contains(lower(dstName), 'terminator')
                delete_line(lineH);
                fprintf('      [OK] Deleted line to Terminator "%s"\n', dstName);
            else
                fprintf('      [INFO] Existing destination is "%s" (not a Terminator)\n', dstName);
            end
        else
            fprintf('      [INFO] No existing line on output port\n');
        end
    catch ME
        fprintf('      [WARN] Could not inspect/delete existing line: %s\n', ME.message);
    end
    
    % --- Add To Workspace block ---
    twFullPath = [modelName '/' twBlockNames{i}];
    try
        already = find_system(modelName, 'SearchDepth', 1, 'Name', twBlockNames{i});
        if ~isempty(already)
            fprintf('      [SKIP] To Workspace block already exists\n');
        else
            add_block('simulink/Sinks/To Workspace', twFullPath, ...
                'VariableName', logVarNames{i}, ...
                'SaveFormat',   'Array', ...
                'MaxDataPoints', 'inf', ...
                'Position', [700, 80 + (i-1)*80, 780, 100 + (i-1)*80]);
            fprintf('      [OK] Added To Workspace block "%s"\n', twBlockNames{i});
            fprintf('           Workspace variable: %s\n', logVarNames{i});
        end
    catch ME
        fprintf('      [WARN] Could not add To Workspace block: %s\n', ME.message);
    end
    
    % --- Connect FromWorkspace --> To Workspace ---
    try
        add_line(modelName, ...
            [fwRel          '/1'], ...
            [twBlockNames{i} '/1'], ...
            'autorouting', 'on');
        fprintf('      [OK] Connected\n\n');
    catch ME
        fprintf('      [SKIP] %s\n\n', ME.message);
    end
    
    connIdx = connIdx + 1;
end

%% -------------------------------------------------------------------------
%  Save model
%% -------------------------------------------------------------------------
fprintf('--- Saving model ---\n');
try
    save_system(modelName);
    fprintf('[OK] Saved: %s.slx\n', modelName);
catch ME
    fprintf('[ERROR] Could not save: %s\n', ME.message);
    fprintf('[INFO]  Use Ctrl+S in Simulink to save manually.\n');
end

%% -------------------------------------------------------------------------
%  Summary
%% -------------------------------------------------------------------------
fprintf('\n============================================================\n');
fprintf(' Connection repair complete. Summary:\n');
fprintf('   [1]  SD/1 --> KALMAN FILTER/1  (position truth to KF)\n');
fprintf('   [2]  SD/1 --> GNSS/1           (position truth to GNSS)\n');
fprintf('   [3a] UKF R input port enabled\n');
fprintf('   [3b] New Inport GNSS_Cov_R added at KF port 4\n');
fprintf('   [3c] GNSS_Cov_R --> UKF/3 (inside KF)\n');
fprintf('   [3d] GNSS/3 --> KALMAN FILTER/4 (root)\n');
fprintf('   [4]  FromWorkspace  --> ToWS_trace_x\n');
fprintf('   [5]  FromWorkspace1 --> ToWS_trace_y\n');
fprintf('   [6]  FromWorkspace2 --> ToWS_trace_z\n');
fprintf('\n NEXT STEPS:\n');
fprintf('   1. Run model for 20,000 s and check Data Inspector for:\n');
fprintf('      - INSTRUMENT DECISION:1  (lambda toggling)\n');
fprintf('      - trace_replay_x/y/z     (heritage GNSS replay active)\n');
fprintf('   2. Verify UKF covariance Cov_act responds to GNSS Cov_matrix\n');
fprintf('   3. Confirm no algebraic loops (Simulation > Update Diagram)\n');
fprintf('============================================================\n');

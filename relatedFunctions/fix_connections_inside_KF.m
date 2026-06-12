%% fix_connections_inside_KF.m
% =========================================================================
% INOAS - INSIDE KALMAN FILTER SUBSYSTEM connections ONLY
% Model: MPCcontrolledSpacecraft_plant_gnss_with_predictor
%
% Run this script INDEPENDENTLY from fix_connections_root.m
% Run this BEFORE fix_connections_root.m (it creates port 4 on KF boundary)
%
% Connections added INSIDE the KALMAN FILTER subsystem:
%   [3a] UKF: set HasMeasurementNoiseCovariancePort = on
%             (adds port in:3 on the UKF block for dynamic R input)
%
%   [3b] Add new Inport block 'GNSS_Cov_R' at port 4 inside KALMAN FILTER
%             (creates the subsystem boundary port that root level will use)
%
%   [3c] GNSS_Cov_R/out:1 --> Unscented Kalman Filter_/in:3
%             (routes the GNSS measurement noise covariance to UKF R input)
%
% =========================================================================

modelName = 'MPCcontrolledSpacecraft_plant_gnss_with_predictor';

%% -------------------------------------------------------------------------
%  0. Load model
%% -------------------------------------------------------------------------
fprintf('============================================================\n');
fprintf(' fix_connections_inside_KF.m  (INSIDE KALMAN FILTER only)\n');
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

%% -------------------------------------------------------------------------
%  1. Locate KALMAN FILTER subsystem and UKF block
%% -------------------------------------------------------------------------
fprintf('\n--- Locating blocks inside KALMAN FILTER ---\n');

% KALMAN FILTER subsystem path
tmp = find_system(modelName, 'SearchDepth', 1, 'Name', 'KALMAN FILTER');
assert(~isempty(tmp), 'ERROR: KALMAN FILTER subsystem not found at root level.');
kfPath = tmp{1};
fprintf('  [OK] KALMAN FILTER path: "%s"\n', kfPath);

% UKF block inside KALMAN FILTER
ukfCandidates = find_system(kfPath, 'SearchDepth', 1, ...
    'RegExp', 'on', 'Name', 'Unscented.*Kalman');
assert(~isempty(ukfCandidates), 'ERROR: UKF block not found inside KALMAN FILTER.');
ukfPath    = ukfCandidates{1};
ukfRelToKF = strrep(ukfPath, [kfPath '/'], '');
fprintf('  [OK] UKF block: "%s"\n', ukfRelToKF);

% Report current UKF port count
ph = get_param(ukfPath, 'PortHandles');
fprintf('  [INFO] UKF currently has %d input ports and %d output ports\n', ...
    numel(ph.Inport), numel(ph.Outport));

fprintf('\n--- Applying connections inside KALMAN FILTER ---\n\n');

%% -------------------------------------------------------------------------
%  STEP [3a]
%  Enable UKF measurement noise covariance input port
%
%  WHY: The UKF by default has 2 inputs (u, y). Setting
%       HasMeasurementNoiseCovariancePort to 'on' adds port in:3 for R,
%       allowing the GNSS covariance to update measurement noise at runtime.
%       This is what makes the UKF trust GNSS more or less depending on
%       satellite geometry (Section 5.4 of INOAS report).
%% -------------------------------------------------------------------------
fprintf('[3a] Enable UKF R input port (HasMeasurementNoiseCovariancePort)\n');
try
    currentVal = get_param(ukfPath, 'HasMeasurementNoiseCovariancePort');
    if strcmp(currentVal, 'on')
        fprintf('     [SKIP] Already enabled\n\n');
    else
        set_param(ukfPath, 'HasMeasurementNoiseCovariancePort', 'on');
        % Verify new port count
        ph = get_param(ukfPath, 'PortHandles');
        fprintf('     [OK] Enabled — UKF now has %d input ports\n', numel(ph.Inport));
        fprintf('          Port in:1 = u (control/IMU forces)\n');
        fprintf('          Port in:2 = y (measurements Z)\n');
        fprintf('          Port in:3 = R (measurement noise covariance)  <-- NEW\n\n');
    end
catch ME
    fprintf('     [ERROR] %s\n', ME.message);
    fprintf('     [MANUAL] Open the UKF block dialog and enable:\n');
    fprintf('              "Use measurement noise covariance port (R)"\n\n');
end

%% -------------------------------------------------------------------------
%  STEP [3b]
%  Add new Inport 'GNSS_Cov_R' at port 4 inside KALMAN FILTER
%
%  WHY: This creates the subsystem boundary port (port 4) that the root-
%       level script will connect GNSS out:3 to. Without this inport the
%       root-level connection [3] in fix_connections_root.m cannot be made.
%% -------------------------------------------------------------------------
newInportName = 'GNSS_Cov_R';
newInportFull = [kfPath '/' newInportName];

fprintf('[3b] Add Inport "%s" at port 4 inside KALMAN FILTER\n', newInportName);
try
    existing = find_system(kfPath, 'SearchDepth', 1, 'Name', newInportName);
    if ~isempty(existing)
        fprintf('     [SKIP] Inport "%s" already exists\n\n', newInportName);
    else
        % Position it below the existing 3 inports
        % (existing inports are approx at y=168, 228 — place new one at 288)
        add_block('simulink/Sources/In1', newInportFull, ...
            'Port',     '4', ...
            'Position', [-755, 288, -725, 302]);
        fprintf('     [OK] Inport "%s" added at port 4\n', newInportName);
        fprintf('          This creates KALMAN FILTER/in:4 at the subsystem boundary.\n\n');
    end
catch ME
    fprintf('     [ERROR] %s\n\n', ME.message);
end

%% -------------------------------------------------------------------------
%  STEP [3c]
%  GNSS_Cov_R/out:1 --> Unscented Kalman Filter_/in:3
%
%  WHY: Wires the new inport to the UKF R input port created in step [3a].
%       This is the internal path that carries GNSS covariance to the UKF.
%       The external path (GNSS subsystem --> KF boundary) is handled by
%       fix_connections_root.m step [3].
%% -------------------------------------------------------------------------
fprintf('[3c] GNSS_Cov_R/1 --> %s/3  (inside KALMAN FILTER)\n', ukfRelToKF);
try
    add_line(kfPath, ...
        [newInportName  '/1'], ...
        [ukfRelToKF     '/3'], ...
        'autorouting', 'on');
    fprintf('     [OK] Connected inside KALMAN FILTER\n\n');
catch ME
    fprintf('     [SKIP] %s\n', ME.message);
    if contains(ME.message, '3')
        fprintf('     [HINT] UKF port 3 does not exist.\n');
        fprintf('            Step [3a] may have failed — check manually.\n');
    end
    fprintf('\n');
end

%% -------------------------------------------------------------------------
%  Verify final port state inside KALMAN FILTER
%% -------------------------------------------------------------------------
fprintf('--- Verification ---\n');
fprintf('KALMAN FILTER inports:\n');
inports = find_system(kfPath, 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(inports)
    pName = get_param(inports{i}, 'Name');
    pNum  = get_param(inports{i}, 'Port');
    fprintf('  Port %s: %s\n', pNum, pName);
end

fprintf('\nUKF port count:\n');
try
    ph = get_param(ukfPath, 'PortHandles');
    fprintf('  Inputs : %d\n', numel(ph.Inport));
    fprintf('  Outputs: %d\n', numel(ph.Outport));
catch
    fprintf('  [WARN] Could not read port handles\n');
end

%% -------------------------------------------------------------------------
%  Save
%% -------------------------------------------------------------------------
fprintf('\n--- Saving model ---\n');
try
    save_system(modelName);
    fprintf('[OK] Saved: %s.slx\n', modelName);
catch ME
    fprintf('[ERROR] %s\n', ME.message);
end

fprintf('\n============================================================\n');
fprintf(' KALMAN FILTER internal connections done.\n');
fprintf(' NOW run fix_connections_root.m to complete step [3].\n');
fprintf('============================================================\n');

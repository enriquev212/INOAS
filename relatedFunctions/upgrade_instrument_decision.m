%% upgrade_instrument_decision.m
% =========================================================================
% INOAS - Replace old INSTRUMENT DECISION with new INSTRUMENT DECISION1
%
% Source model : GNSS_MPCcontrolledSpacecraft.slx  (new, correct wiring)
% Target model : MPCcontrolledSpacecraft_plant_gnss_with_predictor.slx
%
% What this script does:
%   1. Loads both models
%   2. Copies INSTRUMENT DECISION1 block from source to target
%   3. Removes old INSTRUMENT DECISION block and its wires
%   4. Adds VectorConcatenate1 (KF pos+vel) and VectorConcatenate2 (GNSS pos+vel)
%   5. Wires all 4 inputs of the new block:
%        in:1 <- VectorConcatenate2  (X_gnss: GNSS pos+vel)
%        in:2 <- GNSS Subsystem/3   (Q_gnss: GNSS covariance)
%        in:3 <- VectorConcatenate1  (X_est:  KF pos+vel)
%        in:4 <- KALMAN FILTER/3    (Cov_act: KF covariance)
%   6. Wires the output:
%        out:1 (lamda) -> Unit Delay1/in:1
%
% WHY the new block is better:
%   Old block: only used Cov_act (soft threshold only)
%   New block: adds NIS (Normalized Innovation Squared) as a HARD FLAG
%              that immediately forces KF mode if GNSS is statistically
%              inconsistent, regardless of covariance magnitude.
%              This directly implements the satellite validity check
%              described in Section 3.2 of the INOAS report.
%
% RUN ORDER: Run this script standalone. No other script needed first.
%
% =========================================================================

srcModel = 'GNSS_MPCcontrolledSpacecraft';
tgtModel = 'MPCcontrolledSpacecraft_plant_gnss_with_predictor';

%% -------------------------------------------------------------------------
%  0. Load both models
%% -------------------------------------------------------------------------
fprintf('============================================================\n');
fprintf(' upgrade_instrument_decision.m\n');
fprintf(' Source: %s\n', srcModel);
fprintf(' Target: %s\n', tgtModel);
fprintf('============================================================\n\n');

if ~bdIsLoaded(srcModel)
    load_system(srcModel);
    fprintf('[LOAD] Source model loaded.\n');
else
    fprintf('[LOAD] Source model already loaded.\n');
end

if ~bdIsLoaded(tgtModel)
    load_system(tgtModel);
    fprintf('[LOAD] Target model loaded.\n');
else
    fprintf('[LOAD] Target model already loaded.\n');
end

% Set both to normal mode
for mdl = {srcModel, tgtModel}
    if strcmp(get_param(mdl{1}, 'SimulationMode'), 'accelerator')
        set_param(mdl{1}, 'SimulationMode', 'normal');
    end
end

%% -------------------------------------------------------------------------
%  Helper: get position of an existing block
%% -------------------------------------------------------------------------
getPos = @(blk) get_param(blk, 'Position');

%% -------------------------------------------------------------------------
%  1. Locate required blocks in TARGET model
%% -------------------------------------------------------------------------
fprintf('\n--- Locating blocks in target model ---\n');

% Old INSTRUMENT DECISION
oldID_candidates = find_system(tgtModel, 'SearchDepth', 1, 'Name', 'INSTRUMENT DECISION');
assert(~isempty(oldID_candidates), 'ERROR: Old INSTRUMENT DECISION not found in target.');
oldIDPath = oldID_candidates{1};
oldIDPos  = getPos(oldIDPath);
fprintf('  [OK] Old INSTRUMENT DECISION at position: [%d %d %d %d]\n', oldIDPos);

% KALMAN FILTER
tmp = find_system(tgtModel, 'SearchDepth', 1, 'Name', 'KALMAN FILTER');
assert(~isempty(tmp), 'ERROR: KALMAN FILTER not found.');
kfRel = strrep(tmp{1}, [tgtModel '/'], '');
fprintf('  [OK] KALMAN FILTER: "%s"\n', kfRel);

% GNSS Enabled Subsystem
gnssPath = '';
allSubs = find_system(tgtModel, 'SearchDepth', 1, 'BlockType', 'SubSystem');
for k = 1:numel(allSubs)
    if ~isempty(find_system(allSubs{k}, 'SearchDepth', 1, 'BlockType', 'EnablePort'))
        gnssPath = allSubs{k};
        break;
    end
end
assert(~isempty(gnssPath), 'ERROR: GNSS Enabled Subsystem not found.');
gnssRel = strrep(gnssPath, [tgtModel '/'], '');
fprintf('  [OK] GNSS Subsystem: "%s"\n', strrep(gnssRel, newline, '\n'));

% Unit Delay1 (receives lamda output)
tmp = find_system(tgtModel, 'SearchDepth', 1, 'Name', 'Unit Delay1');
assert(~isempty(tmp), 'ERROR: Unit Delay1 not found.');
ud1Rel = strrep(tmp{1}, [tgtModel '/'], '');
fprintf('  [OK] Unit Delay1: "%s"\n', ud1Rel);

% Existing VectorConcatenate (old single one — will be reused as VectorConcatenate2)
tmp = find_system(tgtModel, 'SearchDepth', 1, 'Name', 'Vector\nConcatenate');
if ~isempty(tmp)
    vcOldRel = strrep(tmp{1}, [tgtModel '/'], '');
    fprintf('  [OK] Existing VectorConcatenate: "%s"\n', ...
        strrep(vcOldRel, newline, '\n'));
else
    vcOldRel = '';
    fprintf('  [INFO] No existing VectorConcatenate found.\n');
end

%% -------------------------------------------------------------------------
%  2. Step 1: Delete lines connected TO old INSTRUMENT DECISION
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 1] Deleting lines connected to old INSTRUMENT DECISION ---\n');

ph = get_param(oldIDPath, 'PortHandles');

% Delete all input lines
for i = 1:numel(ph.Inport)
    lh = get_param(ph.Inport(i), 'Line');
    if lh ~= -1
        delete_line(lh);
        fprintf('  [OK] Deleted input line on port %d\n', i);
    end
end

% Delete all output lines
for i = 1:numel(ph.Outport)
    lh = get_param(ph.Outport(i), 'Line');
    if lh ~= -1
        delete_line(lh);
        fprintf('  [OK] Deleted output line on port %d\n', i);
    end
end

%% -------------------------------------------------------------------------
%  3. Step 2: Delete old INSTRUMENT DECISION block
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 2] Deleting old INSTRUMENT DECISION ---\n');
delete_block(oldIDPath);
fprintf('  [OK] Old INSTRUMENT DECISION deleted.\n');

%% -------------------------------------------------------------------------
%  4. Step 3: Copy INSTRUMENT DECISION1 from source to target
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 3] Copying INSTRUMENT DECISION1 from source model ---\n');

srcIDPath = [srcModel '/INSTRUMENT DECISION1'];
tgtIDPath = [tgtModel '/INSTRUMENT DECISION1'];

% Place it at the same position as the old block
newPos = [oldIDPos(1), oldIDPos(2), oldIDPos(1)+220, oldIDPos(2)+120];

try
    already = find_system(tgtModel, 'SearchDepth', 1, 'Name', 'INSTRUMENT DECISION1');
    if ~isempty(already)
        fprintf('  [SKIP] INSTRUMENT DECISION1 already exists in target.\n');
        tgtIDPath = already{1};
    else
        add_block(srcIDPath, tgtIDPath, ...
            'Position', newPos, ...
            'CopyOption', 'duplicate');
        fprintf('  [OK] INSTRUMENT DECISION1 copied to target model.\n');
        fprintf('       Position: [%d %d %d %d]\n', newPos);
    end
catch ME
    fprintf('  [ERROR] %s\n', ME.message);
    fprintf('  [HINT] Make sure both .slx files are in the same folder.\n');
    rethrow(ME);
end

tgtIDRel = strrep(tgtIDPath, [tgtModel '/'], '');

%% -------------------------------------------------------------------------
%  5. Step 4: Add VectorConcatenate1  (KF: pos + vel → X_est)
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 4] Adding VectorConcatenate1 (KF pos+vel) ---\n');

vc1Name = 'VectorConcatenate1';
vc1Path = [tgtModel '/' vc1Name];
vc1Pos  = [oldIDPos(1)-250, oldIDPos(2)-80, oldIDPos(1)-180, oldIDPos(2)-40];

try
    already = find_system(tgtModel, 'SearchDepth', 1, 'Name', vc1Name);
    if ~isempty(already)
        fprintf('  [SKIP] %s already exists.\n', vc1Name);
    else
        add_block('simulink/Math Operations/Vector Concatenate', vc1Path, ...
            'NumInputs', '2', ...
            'Position',  vc1Pos);
        fprintf('  [OK] %s added.\n', vc1Name);
    end
catch ME
    fprintf('  [WARN] %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%  6. Step 5: Add VectorConcatenate2  (GNSS: pos + vel → X_gnss)
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 5] Adding VectorConcatenate2 (GNSS pos+vel) ---\n');

vc2Name = 'VectorConcatenate2';
vc2Path = [tgtModel '/' vc2Name];
vc2Pos  = [oldIDPos(1)-250, oldIDPos(2)+40, oldIDPos(1)-180, oldIDPos(2)+80];

try
    already = find_system(tgtModel, 'SearchDepth', 1, 'Name', vc2Name);
    if ~isempty(already)
        fprintf('  [SKIP] %s already exists.\n', vc2Name);
    else
        add_block('simulink/Math Operations/Vector Concatenate', vc2Path, ...
            'NumInputs', '2', ...
            'Position',  vc2Pos);
        fprintf('  [OK] %s added.\n', vc2Name);
    end
catch ME
    fprintf('  [WARN] %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%  7. Step 6: Wire VectorConcatenate1 (KF pos+vel)
%     KALMAN FILTER/out:1 (pos) --> VectorConcatenate1/in:1
%     KALMAN FILTER/out:2 (vel) --> VectorConcatenate1/in:2
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 6] Wiring VectorConcatenate1 (KF pos+vel) ---\n');

try
    add_line(tgtModel, [kfRel '/1'], [vc1Name '/1'], 'autorouting', 'on');
    fprintf('  [OK] KALMAN FILTER/1 --> VectorConcatenate1/1  (pos)\n');
catch ME
    fprintf('  [SKIP] %s\n', ME.message);
end

try
    add_line(tgtModel, [kfRel '/2'], [vc1Name '/2'], 'autorouting', 'on');
    fprintf('  [OK] KALMAN FILTER/2 --> VectorConcatenate1/2  (vel)\n');
catch ME
    fprintf('  [SKIP] %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%  8. Step 7: Wire VectorConcatenate2 (GNSS pos+vel)
%     GNSS Subsystem/out:1 (pos) --> VectorConcatenate2/in:1
%     GNSS Subsystem/out:2 (vel) --> VectorConcatenate2/in:2
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 7] Wiring VectorConcatenate2 (GNSS pos+vel) ---\n');

try
    add_line(tgtModel, [gnssRel '/1'], [vc2Name '/1'], 'autorouting', 'on');
    fprintf('  [OK] GNSS/1 --> VectorConcatenate2/1  (pos)\n');
catch ME
    fprintf('  [SKIP] %s\n', ME.message);
end

try
    add_line(tgtModel, [gnssRel '/2'], [vc2Name '/2'], 'autorouting', 'on');
    fprintf('  [OK] GNSS/2 --> VectorConcatenate2/2  (vel)\n');
catch ME
    fprintf('  [SKIP] %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%  9. Step 8: Wire the 4 inputs of INSTRUMENT DECISION1
%
%     in:1 <- VectorConcatenate2/out:1  (X_gnss: GNSS pos+vel)
%     in:2 <- GNSS Subsystem/out:3      (Q_gnss: GNSS covariance)
%     in:3 <- VectorConcatenate1/out:1  (X_est:  KF pos+vel)
%     in:4 <- KALMAN FILTER/out:3       (Cov_act: KF covariance)
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 8] Wiring 4 inputs of INSTRUMENT DECISION1 ---\n');

connections_in = {
    [vc2Name '/1'],   [tgtIDRel '/1'],   'X_gnss  (GNSS pos+vel)';
    [gnssRel '/3'],   [tgtIDRel '/2'],   'Q_gnss  (GNSS covariance)';
    [vc1Name '/1'],   [tgtIDRel '/3'],   'X_est   (KF pos+vel)';
    [kfRel '/3'],     [tgtIDRel '/4'],   'Cov_act (KF covariance)';
};

for i = 1:size(connections_in, 1)
    src = connections_in{i,1};
    dst = connections_in{i,2};
    lbl = connections_in{i,3};
    try
        add_line(tgtModel, src, dst, 'autorouting', 'on');
        fprintf('  [OK] in:%d  %s\n', i, lbl);
    catch ME
        fprintf('  [SKIP] in:%d %s -- %s\n', i, lbl, ME.message);
    end
end

%% -------------------------------------------------------------------------
%  10. Step 9: Wire output of INSTRUMENT DECISION1
%      out:1 (lamda) --> Unit Delay1/in:1
%% -------------------------------------------------------------------------
fprintf('\n--- [Step 9] Wiring INSTRUMENT DECISION1 output (lamda) ---\n');

try
    add_line(tgtModel, [tgtIDRel '/1'], [ud1Rel '/1'], 'autorouting', 'on');
    fprintf('  [OK] INSTRUMENT DECISION1/1 --> Unit Delay1/1  (lamda)\n');
catch ME
    fprintf('  [SKIP] %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%  11. Save
%% -------------------------------------------------------------------------
fprintf('\n--- Saving target model ---\n');
try
    save_system(tgtModel);
    fprintf('[OK] Saved: %s.slx\n', tgtModel);
catch ME
    fprintf('[ERROR] %s\n', ME.message);
    fprintf('[INFO]  Use Ctrl+S in Simulink to save manually.\n');
end

%% -------------------------------------------------------------------------
%  Summary
%% -------------------------------------------------------------------------
fprintf('\n============================================================\n');
fprintf(' Upgrade complete. What changed:\n\n');
fprintf(' REMOVED:\n');
fprintf('   - Old INSTRUMENT DECISION (2 inputs, TRACE only)\n\n');
fprintf(' ADDED:\n');
fprintf('   - VectorConcatenate1     (KF pos+vel for X_est)\n');
fprintf('   - VectorConcatenate2     (GNSS pos+vel for X_gnss)\n');
fprintf('   - INSTRUMENT DECISION1  (4 inputs: NIS + TRACE logic)\n\n');
fprintf(' NEW WIRING:\n');
fprintf('   GNSS/1+2    --> VectorConcatenate2 --> INSTR_DEC1/in:1 (X_gnss)\n');
fprintf('   GNSS/3      ---------------------  --> INSTR_DEC1/in:2 (Q_gnss)\n');
fprintf('   KF/1+2      --> VectorConcatenate1 --> INSTR_DEC1/in:3 (X_est)\n');
fprintf('   KF/3        ---------------------  --> INSTR_DEC1/in:4 (Cov_act)\n');
fprintf('   INSTR_DEC1/1 -------------------- --> Unit Delay1/1   (lamda)\n\n');
fprintf(' NEXT STEPS:\n');
fprintf('   1. Run: Simulation > Update Diagram  (check for algebraic loops)\n');
fprintf('   2. Run for 20,000 s, monitor in Data Inspector:\n');
fprintf('      - INSTRUMENT DECISION1 > lamda  (should toggle)\n');
fprintf('      - INSTRUMENT DECISION1 > Scope1 (NIS score)\n');
fprintf('   3. Verify NIS triggers KF mode before TRACE does\n');
fprintf('============================================================\n');

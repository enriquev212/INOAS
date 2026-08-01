function setup_instrument_decision(varargin)
%SETUP_INSTRUMENT_DECISION  Install the new Instrument Decision Module
% =========================================================================
%   Programmatically:
%     1. Writes instrument_decision.m   (the state-machine function)
%     2. Extracts NSV / HPE / VPE / PDOP / SOL from the GNSS .dat file
%        and pushes them into the base workspace as timeseries.
%     3. Copies the original Simulink model to a new file
%        ('..._kalman_decision.slx') and rewires the
%        'INSTRUMENT DECISION1' subsystem with the new logic.
%
%   The subsystem's external interface (4 Inports, 1 Outport) is preserved
%   so the parent-level wiring (Unit Delay1, Switch1/2, Enabled Subsystem)
%   is NOT touched.  The new content uses:
%       - Cov_act        (the existing input)  → J observable
%       - 5 From-Workspace blocks for GNSS quality flags
%       - 1 MATLAB Function block = instrument_decision()
%
% USAGE
%   >> setup_instrument_decision
%   >> setup_instrument_decision('Model', 'my_model_name', 'DatFile', 'foo.dat')
%
%   After running, open the new model and simulate as usual.
% =========================================================================

% ---------------- 1. Parse options --------------------------------------
p = inputParser;
p.addParameter('Model',   'MPCcontrolledSpacecraft_plant_gnss_with_predictor', @ischar);
p.addParameter('NewModel','MPCcontrolledSpacecraft_plant_gnss_kalman_decision',@ischar);
p.addParameter('DatFile', 'cov_perturb_POS_s6a_Y24D011_fixed.dat',      @ischar);
p.addParameter('T_gnss_on', 90,  @isnumeric);   % [s] GNSS duty-cycle ON time
p.addParameter('T_kalman',  10,  @isnumeric);   % [s] Kalman duty-cycle period
p.addParameter('MaxCov',    2000,@isnumeric);   % J threshold
p.parse(varargin{:});
opts = p.Results;

origModel = opts.Model;
newModel  = opts.NewModel;
datFile   = opts.DatFile;

fprintf('\n==== Instrument Decision Module Setup ====\n');
fprintf(' Source model      : %s.slx\n', origModel);
fprintf(' Destination model : %s.slx\n', newModel);
fprintf(' GNSS .dat file    : %s\n', datFile);

% ---------------- 2. Write the function files --------------------------
write_instrument_decision_file();
fprintf(' [1/4] instrument_decision.m written.\n');

write_load_gnss_function_file();
fprintf('       load_gnss_quality_signals.m written.\n');

% ---------------- 3. Extract GNSS quality signals into workspace --------
load_gnss_quality_signals(datFile);
fprintf(' [2/4] GNSS quality signals loaded into base workspace.\n');

% Push instrument decision parameters to workspace (used by mask if you
% later promote them).
assignin('base', 'T_gnss_on_param', opts.T_gnss_on);
assignin('base', 'T_kalman_param',  opts.T_kalman);
assignin('base', 'max_cov_param',   opts.MaxCov);
% Make sure 'Ts' is in workspace; default to 1 s if missing.
if evalin('base', 'exist(''Ts'',''var'')') == 0
    assignin('base', 'Ts', 1);
end

% ---------------- 4. Duplicate & open the model ------------------------
srcFile = which([origModel '.slx']);
if isempty(srcFile)
    srcFile = [origModel '.slx'];
    if ~isfile(srcFile)
        error('Cannot find %s.slx on the MATLAB path or current folder.', origModel);
    end
end

dstFile = [newModel '.slx'];
if isfile(dstFile)
    % Unload any previously-loaded version before overwriting
    if bdIsLoaded(newModel)
        close_system(newModel, 0);
    end
    delete(dstFile);
end
copyfile(srcFile, dstFile);
fprintf(' [3/4] Model copied to %s\n', dstFile);

load_system(newModel);

% ---------------- 5. Rebuild the INSTRUMENT DECISION1 subsystem --------
modify_instrument_decision_subsystem(newModel);

% ---------------- 6. Install model InitFcn callback --------------------
%   This guarantees that the ts_gnss_* timeseries are reloaded BEFORE
%   every simulation, even if the user's batch script clears the
%   workspace beforehand. The callback is appended to any existing
%   InitFcn so we don't clobber other init code.
install_init_callback(newModel, datFile);
fprintf('       Model InitFcn callback installed.\n');

% Save & finish
save_system(newModel);
fprintf(' [4/4] INSTRUMENT DECISION1 subsystem rewired and saved.\n');
fprintf('==== Done. Open %s and simulate. ====\n\n', newModel);

end % function setup_instrument_decision


% =========================================================================
%                          HELPER FUNCTIONS
% =========================================================================

function write_instrument_decision_file()
%WRITE_INSTRUMENT_DECISION_FILE   Create instrument_decision.m in pwd.
fname = 'instrument_decision.m';
src = {
'function lambda = instrument_decision(J, n_sat, PDOP, HPE, VPE, gnss_sol)'
'% INSTRUMENT_DECISION  State machine: GNSS vs Kalman selector'
'%   Outputs lambda = 1 (GNSS active)  or  0 (Kalman propagation only).'
'%'
'% INPUTS'
'%   J        - trace(S_inv * P * S_T_inv)  covariance observable [scalar]'
'%   n_sat    - number of GNSS satellites used (NSV)'
'%   PDOP     - position dilution of precision'
'%   HPE,VPE  - horizontal / vertical position error [m]'
'%   gnss_sol - GNSS solution validity flag (1 valid, 0 no-fix)'
'%'
'% BEHAVIOUR'
'%   GNSS   -> KALMAN : timer >= T_gnss_on  (periodic duty cycle)'
'%                      OR any Table-3.1 emergency condition'
'%   KALMAN -> GNSS   : timer >= T_kalman_on AND GNSS healthy'
'%                      OR J > max_cov AND GNSS healthy'
'%#codegen'
''
'persistent state timer_count;'
''
'% --- Parameters ----------------------------------------------------------'
'T_gnss_on = 90;   % [s] GNSS ON period before switching to Kalman'
'T_kalman_on = 10; % [s] Kalman ON period before returning to GNSS'
'max_cov   = 2000; % J threshold for forced return to GNSS'
'n_sat_min = 4;    % minimum satellites for a valid fix  (Table 3.1)'
'PDOP_max  = 6;    % PDOP limit                          (Table 3.1)'
'HPE_max   = 5;    % horizontal position error limit [m] (Table 3.1)'
'VPE_max   = 5;    % vertical   position error limit [m] (Table 3.1)'
'Ts        = 1;    % [s] execution sample time of this decision block'
''
'GNSS_STATE   = int32(0);'
'KALMAN_STATE = int32(1);'
''
'if isempty(state)'
'    state       = GNSS_STATE;'
'    timer_count = int32(0);'
'end'
''
'% --- Table-3.1 emergency conditions -------------------------------------'
'signal_outage   = (n_sat < n_sat_min) || (gnss_sol < 0.5);'
'precision_loss  = (HPE > HPE_max)     || (VPE > VPE_max);'
'geometric_drift = (PDOP > PDOP_max);'
'emergency       = signal_outage || precision_loss || geometric_drift;'
''
'timer_count = timer_count + int32(Ts);'
''
'switch state'
'    case GNSS_STATE'
'        if emergency'
'            % Immediate forced switch - GNSS is unusable'
'            state       = KALMAN_STATE;'
'            timer_count = int32(0);'
'        elseif timer_count >= int32(T_gnss_on)'
'            % Periodic duty-cycle: GNSS has been on long enough'
'            state       = KALMAN_STATE;'
'            timer_count = int32(0);'
'        end'
''
'    case KALMAN_STATE'
'        % Real duty-cycle: leave Kalman after its ON window, or earlier if'
'        % the Kalman covariance observable becomes too large.'
'        gnss_healthy = ~emergency;'
'        if gnss_healthy && ((timer_count >= int32(T_kalman_on)) || (J > max_cov))'
'            state       = GNSS_STATE;'
'            timer_count = int32(0);'
'        end'
''
'    otherwise'
'        state       = GNSS_STATE;'
'        timer_count = int32(0);'
'end'
''
'lambda = double(state == GNSS_STATE);'
''
'end'
};
fid = fopen(fname, 'w');
fprintf(fid, '%s\n', src{:});
fclose(fid);
end


function write_load_gnss_function_file()
%WRITE_LOAD_GNSS_FUNCTION_FILE  Create load_gnss_quality_signals.m in pwd.
%   This file is needed because the model's InitFcn calls it before every
%   simulation, so it must exist as a standalone function on the path.
fname = 'load_gnss_quality_signals.m';
src = {
'function load_gnss_quality_signals(datFile)'
'%LOAD_GNSS_QUALITY_SIGNALS  Push NSV/HPE/VPE/PDOP/SOL into base workspace.'
'%   Called by the Simulink model InitFcn before every simulation to ensure'
'%   that the From-Workspace blocks inside INSTRUMENT DECISION1 find their'
'%   data, even if the user batch script has just cleared the workspace.'
'%'
'%   Column map of the .dat file:'
'%     1=SOD 2=LON 3=LAT 4=ALT 5=CLK 6=GGTO 7=SOL 8=NSVVIS 9=NSV'
'%     10=HPE 11=VPE 12=EPE 13=NPE 14=UPE 15=HDOP 16=VDOP 17=PDOP'
''
'if nargin < 1 || isempty(datFile)'
'    datFile = ''cov_perturb_POS_s6a_Y24D011_fixed.dat'';'
'end'
''
'datFile = inoas_data_file(datFile);'
''
'if ~isfile(datFile)'
'    error(''load_gnss_quality_signals: file not found: %s'', datFile);'
'end'
''
'fid = fopen(datFile, ''r'');'
'raw = textscan(fid, repmat(''%f'',1,17), ''HeaderLines'', 1);'
'fclose(fid);'
''
't_dat = raw{1};'
''
'ts_gnss_sol  = timeseries(double(raw{7}),  t_dat, ''Name'', ''gnss_sol'');'
'ts_gnss_nsv  = timeseries(double(raw{9}),  t_dat, ''Name'', ''gnss_nsv'');'
'ts_gnss_hpe  = timeseries(double(raw{10}), t_dat, ''Name'', ''gnss_hpe'');'
'ts_gnss_vpe  = timeseries(double(raw{11}), t_dat, ''Name'', ''gnss_vpe'');'
'ts_gnss_pdop = timeseries(double(raw{17}), t_dat, ''Name'', ''gnss_pdop'');'
''
'assignin(''base'', ''ts_gnss_sol'',  ts_gnss_sol);'
'assignin(''base'', ''ts_gnss_nsv'',  ts_gnss_nsv);'
'assignin(''base'', ''ts_gnss_hpe'',  ts_gnss_hpe);'
'assignin(''base'', ''ts_gnss_vpe'',  ts_gnss_vpe);'
'assignin(''base'', ''ts_gnss_pdop'', ts_gnss_pdop);'
''
'if evalin(''base'', ''exist(''''Ts'''',''''var'''')'') == 0'
'    assignin(''base'', ''Ts'', 1);'
'end'
''
'end'
};
fid = fopen(fname, 'w');
fprintf(fid, '%s\n', src{:});
fclose(fid);
end


function install_init_callback(modelName, datFile)
%INSTALL_INIT_CALLBACK  Set the model InitFcn to auto-load GNSS signals.
%   Appended to any pre-existing InitFcn code so we do not destroy it.

% Use forward slashes to make the path robust on any OS
datFile = strrep(datFile, '\', '/');

newSnippet = sprintf([ ...
    '%%--- AUTO-INSERTED BY setup_instrument_decision (do not edit by hand)\n' ...
    'if exist(''load_gnss_quality_signals'', ''file'') == 2\n' ...
    '    load_gnss_quality_signals(''%s'');\n' ...
    'end\n' ...
    '%%--- END AUTO-INSERTED\n'], datFile);

oldInit = get_param(modelName, 'InitFcn');

% Strip any previous auto-inserted block (idempotent re-install)
pattern = '%--- AUTO-INSERTED BY setup_instrument_decision[\s\S]*?%--- END AUTO-INSERTED\n?';
oldInit = regexprep(oldInit, pattern, '');

if isempty(strtrim(oldInit))
    newInit = newSnippet;
else
    newInit = [oldInit newline newSnippet];
end

set_param(modelName, 'InitFcn', newInit);
end


function load_gnss_quality_signals(datFile)
%LOAD_GNSS_QUALITY_SIGNALS  (local copy used during setup; mirrors the
%standalone file written above so the first run works even before the file
%is on the path).

datFile = inoas_data_file(datFile);

if ~isfile(datFile)
    error('Could not find GNSS data file: %s', datFile);
end

fid = fopen(datFile, 'r');
raw = textscan(fid, repmat('%f',1,17), 'HeaderLines', 1);
fclose(fid);

t_dat = raw{1};

ts_gnss_sol  = timeseries(double(raw{7}),  t_dat,  'Name', 'gnss_sol');
ts_gnss_nsv  = timeseries(double(raw{9}),  t_dat,  'Name', 'gnss_nsv');
ts_gnss_hpe  = timeseries(double(raw{10}), t_dat,  'Name', 'gnss_hpe');
ts_gnss_vpe  = timeseries(double(raw{11}), t_dat,  'Name', 'gnss_vpe');
ts_gnss_pdop = timeseries(double(raw{17}), t_dat,  'Name', 'gnss_pdop');

assignin('base', 'ts_gnss_sol',  ts_gnss_sol);
assignin('base', 'ts_gnss_nsv',  ts_gnss_nsv);
assignin('base', 'ts_gnss_hpe',  ts_gnss_hpe);
assignin('base', 'ts_gnss_vpe',  ts_gnss_vpe);
assignin('base', 'ts_gnss_pdop', ts_gnss_pdop);

% S_inv / S_T_inv for the J observable (only if not already set by user)
if evalin('base', 'exist(''S_inv'',''var'')') == 0
    sigma_x = 10; sigma_v = 1;
    S = diag([sigma_x, sigma_x, sigma_x, sigma_v, sigma_v, sigma_v]);
    assignin('base', 'S_inv',   inv(S));
    assignin('base', 'S_T_inv', inv(S.'));
end

fprintf('   NSV  range : [%d  %d]\n',       min(raw{9}),  max(raw{9}));
fprintf('   HPE  range : [%.2f  %.2f] m\n', min(raw{10}), max(raw{10}));
fprintf('   VPE  range : [%.2f  %.2f] m\n', min(raw{11}), max(raw{11}));
fprintf('   PDOP range : [%.2f  %.2f]\n',   min(raw{17}), max(raw{17}));
fprintf('   epochs NSV<4    : %d\n', sum(raw{9}  < 4));
fprintf('   epochs HPE>5    : %d\n', sum(raw{10} > 5));
fprintf('   epochs PDOP>6   : %d\n', sum(raw{17} > 6));
end


function modify_instrument_decision_subsystem(modelName)
%MODIFY_INSTRUMENT_DECISION_SUBSYSTEM  Rebuild INSTRUMENT DECISION1 contents.

subsys = [modelName '/INSTRUMENT DECISION1'];

% Verify subsystem exists
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'INSTRUMENT DECISION1'))
    error('Subsystem ''INSTRUMENT DECISION1'' not found in %s.', modelName);
end

% --- Get existing inports & outport (we keep these) ---------------------
inports  = find_system(subsys, 'SearchDepth', 1, 'BlockType', 'Inport');
outports = find_system(subsys, 'SearchDepth', 1, 'BlockType', 'Outport');

% Build a name-keyed map of inports
inMap = containers.Map();
for k = 1:numel(inports)
    nm = get_param(inports{k}, 'Name');
    inMap(nm) = inports{k};
end

% --- Delete every block inside that is NOT a top-level Inport/Outport ---
allBlocks = find_system(subsys, 'SearchDepth', 1);
allBlocks = allBlocks(~strcmp(allBlocks, subsys));   % drop the subsystem itself
keepers   = [inports; outports];

% Delete lines first (some are still attached to soon-to-be-deleted blocks)
ph = get_param(subsys, 'PortHandles');  % not used directly but forces eval
lines = find_system(subsys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');
for k = 1:numel(lines)
    try, delete_line(lines(k)); catch, end
end

for k = 1:numel(allBlocks)
    if ~any(strcmp(allBlocks{k}, keepers))
        try
            delete_block(allBlocks{k});
        catch ME
            warning('Could not delete %s: %s', allBlocks{k}, ME.message);
        end
    end
end

% --- Reposition inports/outport on a clean grid ------------------------
set_inport_position(inMap, 'X_gnss',  [ 30,  30,  60,  44]);
set_inport_position(inMap, 'Q_gnss',  [ 30,  80,  60,  94]);
set_inport_position(inMap, 'X_est',   [ 30, 130,  60, 144]);
set_inport_position(inMap, 'Cov_act', [ 30, 220,  60, 234]);

% Outport position
if ~isempty(outports)
    set_param(outports{1}, 'Position', [780, 260, 810, 274]);
end

% --- Add Terminators for the 3 unused inputs ---------------------------
add_terminator(subsys, 'Term_X_gnss',  [100,  25, 130,  55], inMap('X_gnss'));
add_terminator(subsys, 'Term_Q_gnss',  [100,  75, 130, 105], inMap('Q_gnss'));
add_terminator(subsys, 'Term_X_est',   [100, 125, 130, 155], inMap('X_est'));

% --- Add the From-Workspace blocks (5 GNSS quality signals) ------------
fwBlocks = {
    'FW_NSV',  'ts_gnss_nsv',  [ 30, 320, 130, 350]
    'FW_PDOP', 'ts_gnss_pdop', [ 30, 370, 130, 400]
    'FW_HPE',  'ts_gnss_hpe',  [ 30, 420, 130, 450]
    'FW_VPE',  'ts_gnss_vpe',  [ 30, 470, 130, 500]
    'FW_SOL',  'ts_gnss_sol',  [ 30, 520, 130, 550]
};
for k = 1:size(fwBlocks,1)
    blkName = [subsys '/' fwBlocks{k,1}];
    add_block('simulink/Sources/From Workspace', blkName, ...
        'Position',     fwBlocks{k,3}, ...
        'VariableName', fwBlocks{k,2}, ...
        'SampleTime',   'Ts');

    % These parameters' valid strings vary across releases. Set them
    % individually and ignore any that this release rejects.
    safe_set_param(blkName, 'Interpolate',           'off');
    safe_set_param(blkName, 'OutputAfterFinalValue', 'Holding final value');
    safe_set_param(blkName, 'ZeroCross',             'off');
end

% --- Add the J-observable MATLAB Function block ------------------------
jblk = [subsys '/Compute J'];
add_block('simulink/User-Defined Functions/MATLAB Function', jblk, ...
    'Position', [220, 210, 360, 270]);
set_mfcn_body(jblk, sprintf([ ...
    'function J = compute_J(P)\n' ...
    '%%#codegen\n' ...
    '%% J = trace(S_inv * P * S_T_inv) with sigma_x=10, sigma_v=1\n' ...
    'S_inv   = diag([0.1,0.1,0.1,1,1,1]);\n' ...
    'S_T_inv = diag([0.1,0.1,0.1,1,1,1]);\n' ...
    'n = numel(P);\n' ...
    'if n == 36\n' ...
    '    Pm = reshape(P,6,6);\n' ...
    'elseif n == 6\n' ...
    '    Pm = diag(P(:));\n' ...
    'elseif n == 1\n' ...
    '    J = double(P);\n' ...
    '    return;\n' ...
    'else\n' ...
    '    Pm = reshape(P,6,6);  %% best effort\n' ...
    'end\n' ...
    'J = trace(S_inv * Pm * S_T_inv);\n' ...
    'end\n']));

% --- Add the Instrument Decision MATLAB Function block ----------------
idblk = [subsys '/Instrument Decision FSM'];
add_block('simulink/User-Defined Functions/MATLAB Function', idblk, ...
    'Position', [480, 360, 680, 480]);
set_mfcn_body(idblk, fileread('instrument_decision.m'));

% Set MATLAB Function block to discrete with Ts
try
    cfg = get_param(idblk, 'MATLABFunctionConfiguration');
    cfg.SampleTime = 'Ts';
catch
    % older release: ignore, sample time inherits from inputs (Ts)
end

% --- Visualization scope (helpful for debugging) -----------------------
add_block('simulink/Sinks/Scope', [subsys '/Lambda Scope'], ...
    'Position', [720, 420, 760, 450], 'NumInputPorts', '1');

% --- Wire everything ---------------------------------------------------
% Cov_act → Compute J
add_line(subsys, 'Cov_act/1', 'Compute J/1', 'autorouting', 'on');

% Compute J → Instrument Decision (input 1 = J)
add_line(subsys, 'Compute J/1', 'Instrument Decision FSM/1', 'autorouting', 'on');

% From-Workspace blocks → Instrument Decision inputs 2..6
add_line(subsys, 'FW_NSV/1',  'Instrument Decision FSM/2', 'autorouting', 'on');
add_line(subsys, 'FW_PDOP/1', 'Instrument Decision FSM/3', 'autorouting', 'on');
add_line(subsys, 'FW_HPE/1',  'Instrument Decision FSM/4', 'autorouting', 'on');
add_line(subsys, 'FW_VPE/1',  'Instrument Decision FSM/5', 'autorouting', 'on');
add_line(subsys, 'FW_SOL/1',  'Instrument Decision FSM/6', 'autorouting', 'on');

% Terminate unused inputs
add_line(subsys, 'X_gnss/1',  'Term_X_gnss/1', 'autorouting', 'on');
add_line(subsys, 'Q_gnss/1',  'Term_Q_gnss/1', 'autorouting', 'on');
add_line(subsys, 'X_est/1',   'Term_X_est/1',  'autorouting', 'on');

% Instrument Decision → Outport lamda
add_line(subsys, 'Instrument Decision FSM/1', 'lamda/1', 'autorouting', 'on');

% Instrument Decision → Lambda Scope (branch off the same signal)
add_line(subsys, 'Instrument Decision FSM/1', 'Lambda Scope/1', 'autorouting', 'on');

end


% --- small wrappers ------------------------------------------------------
function add_terminator(parent, name, pos, srcBlock) %#ok<INUSD>
add_block('simulink/Sinks/Terminator', [parent '/' name], 'Position', pos);
end

function set_inport_position(inMap, name, pos)
if inMap.isKey(name)
    set_param(inMap(name), 'Position', pos);
end
end

function set_mfcn_body(blockPath, codeStr)
%SET_MFCN_BODY  Push code into a MATLAB Function block via Stateflow API.
sf = sfroot();
chart = sf.find('-isa','Stateflow.EMChart','Path', blockPath);
if isempty(chart)
    error('MATLAB Function block not found: %s', blockPath);
end
chart.Script = codeStr;
end


function safe_set_param(blk, paramName, paramValue)
%SAFE_SET_PARAM  set_param wrapper that tolerates unknown/invalid options.
try
    set_param(blk, paramName, paramValue);
catch ME
    warning('safe_set_param: could not set %s=%s on %s (%s)', ...
        paramName, paramValue, blk, ME.message);
end
end

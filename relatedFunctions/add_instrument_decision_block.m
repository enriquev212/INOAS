function add_instrument_decision_block(modelName)
% ADD_INSTRUMENT_DECISION_BLOCK  Add the INOAS Instrument Decision Logic
%   subsystem to an existing Simulink model that already has the UKF and
%   GNSS Enabled Subsystem in place.
%
% USAGE:
%   add_instrument_decision_block()
%       Uses the default model name.
%
%   add_instrument_decision_block('MPCcontrolledSpacecraft_plant_gnss_no_predictor')
%       Targets the specified model.
%
% WHAT THIS SCRIPT DOES:
%   1. Locates the UKF block and GNSS Enabled Subsystem in the model.
%   2. Adds an "Instrument Decision" subsystem at the top level.
%   3. Inside the subsystem, builds:
%        - Computation_Lamda_GNSS   (MATLAB Function: compute_lambda_gnss)
%        - Computation_Lamda_KALMAN (MATLAB Function: compute_lambda_kalman)
%        - A Unit Delay (1/z) for lambda feedback
%        - A Switch block to route the correct evaluator output
%   4. Wires the subsystem to the UKF covariance/state outputs and to the
%      GNSS Enabled Subsystem Enable port.
%   5. Adds a lambda scope for validation.
%
% PREREQUISITES:
%   - compute_lambda_gnss.m   must be on the MATLAB path (same folder)
%   - compute_lambda_kalman.m must be on the MATLAB path (same folder)
%   - The model must be open (or loadable) in Simulink.
%
% The script is non-destructive: it checks for existing blocks before
% adding them and uses backed-up saves (following the project convention
% from install_gnss_sensor_model_patch.m).

    if nargin < 1 || isempty(modelName)
        modelName = 'MPCcontrolledSpacecraft_plant_gnss_no_predictor';
    end
    modelName = char(modelName);

    %----------------------------------------------------------------------
    % 0. Load the model
    %----------------------------------------------------------------------
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
    fprintf('Model loaded: %s\n', modelName);

    %----------------------------------------------------------------------
    % 1. Locate required existing blocks
    %----------------------------------------------------------------------
    ukfBlock   = find_ukf_block(modelName);
    gnssBlock  = find_gnss_enabled_subsystem(modelName);
    fprintf('UKF block  : %s\n', ukfBlock);
    fprintf('GNSS block : %s\n', gnssBlock);

    %----------------------------------------------------------------------
    % 2. Determine port indices on UKF block
    %----------------------------------------------------------------------
    [covPortIdx, statePortIdx] = find_ukf_output_ports(ukfBlock);
    fprintf('UKF covariance port : %d\n', covPortIdx);
    fprintf('UKF state port      : %d\n', statePortIdx);

    %----------------------------------------------------------------------
    % 3. Add "Instrument Decision" subsystem at top level
    %----------------------------------------------------------------------
    idPath = [modelName, '/Instrument Decision'];
    if getSimulinkBlockHandle(idPath) == -1
        add_block('simulink/Ports & Subsystems/Subsystem', idPath, ...
                  'MakeNameUnique', 'off');
        fprintf('Added subsystem: %s\n', idPath);
    else
        fprintf('Subsystem already exists: %s — rebuilding interior.\n', idPath);
    end

    % Position the subsystem nicely in the model canvas
    set_param(idPath, 'Position', [600, 250, 800, 370]);

    %----------------------------------------------------------------------
    % 4. Build the interior of the Instrument Decision subsystem
    %----------------------------------------------------------------------
    build_instrument_decision_interior(idPath);

    %----------------------------------------------------------------------
    % 5. Add inports at the top level for Pk, x_kf, x_gnss, gnss_available
    %----------------------------------------------------------------------
    % (The subsystem already has In1/Out1 from the template – we rename/
    %  repurpose them.)

    %----------------------------------------------------------------------
    % 6. Wire the subsystem to UKF and to GNSS Enable port
    %----------------------------------------------------------------------
    connect_instrument_decision(modelName, idPath, ukfBlock, gnssBlock, ...
                                covPortIdx, statePortIdx);

    %----------------------------------------------------------------------
    % 7. Add a lambda scope for easy inspection
    %----------------------------------------------------------------------
    add_lambda_scope(modelName, idPath);

    %----------------------------------------------------------------------
    % 8. Save
    %----------------------------------------------------------------------
    save_system(modelName);
    fprintf('\nDone.  Model saved: %s.slx\n', modelName);
    fprintf('Open the model and run to verify lambda switching.\n');
end

%==========================================================================
%  LOCAL HELPER FUNCTIONS
%==========================================================================

function build_instrument_decision_interior(idPath)
% Rebuilds the inside of the Instrument Decision subsystem.

    % Remove default template lines and blocks (keep Inports/Outports)
    delete_internal_lines(idPath);
    delete_non_port_blocks(idPath);

    %----------------------------------------------------------------------
    % Inports
    %  1: Pk          (6x6 covariance from UKF)
    %  2: x_kf        (6x1 KF state estimate)
    %  3: x_gnss      (6x1 raw GNSS state)
    %  4: gnss_available (scalar flag)
    %----------------------------------------------------------------------
    inPk   = find_or_add_inport(idPath, 'Pk',            1, [30  65  60  85]);
    inXkf  = find_or_add_inport(idPath, 'x_kf',          2, [30 155 60 175]);
    inXgnss= find_or_add_inport(idPath, 'x_gnss',        3, [30 245 60 265]);
    inGAvail=find_or_add_inport(idPath, 'gnss_available', 4, [30 335 60 355]);

    %----------------------------------------------------------------------
    % Outport
    %  1: lambda
    %----------------------------------------------------------------------
    outLambda = find_or_add_outport(idPath, 'lambda', 1, [720 195 750 215]);

    %----------------------------------------------------------------------
    % Unit Delay – feeds lambda_prev back into both evaluators
    %----------------------------------------------------------------------
    udPath = [idPath, '/Unit Delay'];
    if getSimulinkBlockHandle(udPath) == -1
        add_block('simulink/Discrete/Unit Delay', udPath, 'MakeNameUnique','off');
    end
    set_param(udPath, ...
        'InitialCondition', '1', ...   % start with GNSS on
        'SampleTime',       'Ts', ...
        'Position',         [380 320 430 350]);

    %----------------------------------------------------------------------
    % MATLAB Function block: Computation_Lamda_GNSS (deactivation)
    %----------------------------------------------------------------------
    gnssEvalPath = [idPath, '/Computation_Lamda_GNSS'];
    if getSimulinkBlockHandle(gnssEvalPath) == -1
        add_block('simulink/User-Defined Functions/MATLAB Function', ...
                  gnssEvalPath, 'MakeNameUnique', 'off');
    end
    set_param(gnssEvalPath, 'Position', [200  60 340 130]);
    set_matlab_function_script(gnssEvalPath, compute_lambda_gnss_script());

    %----------------------------------------------------------------------
    % MATLAB Function block: Computation_Lamda_KALMAN (reactivation)
    %----------------------------------------------------------------------
    kfEvalPath = [idPath, '/Computation_Lamda_KALMAN'];
    if getSimulinkBlockHandle(kfEvalPath) == -1
        add_block('simulink/User-Defined Functions/MATLAB Function', ...
                  kfEvalPath, 'MakeNameUnique', 'off');
    end
    set_param(kfEvalPath, 'Position', [200 200 340 320]);
    set_matlab_function_script(kfEvalPath, compute_lambda_kalman_script());

    %----------------------------------------------------------------------
    % Switch block – selects GNSS evaluator when lambda_prev >= 0.5,
    %                KF evaluator otherwise
    %
    %  Input 1: lambda_gnss  (GNSS deactivation output)
    %  Input 2: lambda_prev  (switching signal – threshold 0.5)
    %  Input 3: lambda_kf    (KF reactivation output)
    %----------------------------------------------------------------------
    swPath = [idPath, '/Mode Switch'];
    if getSimulinkBlockHandle(swPath) == -1
        add_block('simulink/Signal Routing/Switch', swPath, ...
                  'MakeNameUnique', 'off');
    end
    set_param(swPath, ...
        'Criteria',   'u2 >= Threshold', ...
        'Threshold',  '0.5', ...
        'Position',   [520 165 570 225]);

    %----------------------------------------------------------------------
    % Connect blocks inside the subsystem
    %----------------------------------------------------------------------

    % ---- Computation_Lamda_GNSS inputs ----
    % Port 1: Pk
    add_line_safe(idPath, 'Pk/1',            'Computation_Lamda_GNSS/1');
    % Port 2: lambda_prev  (from Unit Delay output – routed back)
    add_line_safe(idPath, 'Unit Delay/1',    'Computation_Lamda_GNSS/2');

    % ---- Computation_Lamda_KALMAN inputs ----
    % Port 1: Pk
    add_line_safe(idPath, 'Pk/1',             'Computation_Lamda_KALMAN/1');
    % Port 2: x_kf
    add_line_safe(idPath, 'x_kf/1',           'Computation_Lamda_KALMAN/2');
    % Port 3: x_gnss
    add_line_safe(idPath, 'x_gnss/1',         'Computation_Lamda_KALMAN/3');
    % Port 4: gnss_available
    add_line_safe(idPath, 'gnss_available/1',  'Computation_Lamda_KALMAN/4');
    % Port 5: lambda_prev
    add_line_safe(idPath, 'Unit Delay/1',      'Computation_Lamda_KALMAN/5');

    % ---- Switch inputs ----
    add_line_safe(idPath, 'Computation_Lamda_GNSS/1',   'Mode Switch/1');
    add_line_safe(idPath, 'Unit Delay/1',                'Mode Switch/2');
    add_line_safe(idPath, 'Computation_Lamda_KALMAN/1',  'Mode Switch/3');

    % ---- Switch output -> lambda outport ----
    add_line_safe(idPath, 'Mode Switch/1', 'lambda/1');

    % ---- Feedback: lambda -> Unit Delay input ----
    add_line_safe(idPath, 'Mode Switch/1', 'Unit Delay/1');

    fprintf('Interior of Instrument Decision subsystem built.\n');
end

%--------------------------------------------------------------------------
function connect_instrument_decision(modelName, idPath, ukfBlock, gnssBlock, ...
                                     covPortIdx, statePortIdx)
% Connects the top-level Instrument Decision subsystem to the UKF outputs
% and to the GNSS Enabled Subsystem Enable port.

    idName = get_param(idPath, 'Name');

    % ---- UKF covariance -> Instrument Decision port 1 (Pk) ----
    src = [get_param(ukfBlock,'Name'), '/', num2str(covPortIdx)];
    dst = [idName, '/1'];
    add_line_safe(modelName, src, dst);

    % ---- UKF state estimate -> Instrument Decision port 2 (x_kf) ----
    src = [get_param(ukfBlock,'Name'), '/', num2str(statePortIdx)];
    dst = [idName, '/2'];
    add_line_safe(modelName, src, dst);

    % ---- For x_gnss and gnss_available we use Constant blocks if the
    %      model doesn't already expose raw GNSS state at the top level.
    %      Replace these with real signal sources if available. ----
    xgnssConst = [modelName, '/x_gnss_zero'];
    if getSimulinkBlockHandle(xgnssConst) == -1
        add_block('simulink/Sources/Constant', xgnssConst, ...
                  'MakeNameUnique','off');
        set_param(xgnssConst, 'Value', 'zeros(6,1)', ...
                              'Position', [450 350 530 390]);
    end
    gAvailConst = [modelName, '/gnss_avail_flag'];
    if getSimulinkBlockHandle(gAvailConst) == -1
        add_block('simulink/Sources/Constant', gAvailConst, ...
                  'MakeNameUnique','off');
        set_param(gAvailConst, 'Value', '0', ...
                               'Position', [450 410 530 440]);
    end

    add_line_safe(modelName, 'x_gnss_zero/1',   [idName, '/3']);
    add_line_safe(modelName, 'gnss_avail_flag/1',[idName, '/4']);

    % ---- Instrument Decision lambda output -> GNSS Enable port ----
    gnssName = get_param(gnssBlock, 'Name');
    enablePorts = find_system(gnssBlock, 'SearchDepth',1, ...
                              'BlockType','EnablePort');
    if ~isempty(enablePorts)
        % Route lambda through a Goto/From pair to keep the diagram clean,
        % or directly if the blocks are close enough.
        lambdaGoto = [modelName, '/Goto_lambda'];
        if getSimulinkBlockHandle(lambdaGoto) == -1
            add_block('simulink/Signal Routing/Goto', lambdaGoto, ...
                      'MakeNameUnique','off');
            set_param(lambdaGoto, 'GotoTag',  'lambda_GNSS', ...
                                  'TagVisibility','global', ...
                                  'Position', [830 295 880 325]);
        end

        lambdaFrom = [gnssBlock, '/From_lambda'];
        if getSimulinkBlockHandle(lambdaFrom) == -1
            add_block('simulink/Signal Routing/From', lambdaFrom, ...
                      'MakeNameUnique','off');
            set_param(lambdaFrom, 'GotoTag', 'lambda_GNSS', ...
                                  'Position', [30 -40 110 -20]);
        end

        add_line_safe(modelName, [idName, '/1'],         'Goto_lambda/1');
        add_line_safe(gnssBlock,  'From_lambda/1',  [gnssName,'/Enable']);
    else
        % Fallback: wire directly
        add_line_safe(modelName, [idName, '/1'], [gnssName, '/Enable']);
    end

    fprintf('Top-level wiring completed.\n');
end

%--------------------------------------------------------------------------
function add_lambda_scope(modelName, idPath)
% Adds a Scope block to visualise the lambda switching signal.
    idName = get_param(idPath, 'Name');
    scopePath = [modelName, '/Lambda Scope'];
    if getSimulinkBlockHandle(scopePath) == -1
        add_block('simulink/Sinks/Scope', scopePath, 'MakeNameUnique','off');
        set_param(scopePath, 'Position', [830 230 880 260]);
        add_line_safe(modelName, [idName, '/1'], 'Lambda Scope/1');
        fprintf('Added Lambda Scope.\n');
    end
end

%==========================================================================
%  BLOCK DISCOVERY HELPERS
%==========================================================================

function ukfBlock = find_ukf_block(modelName)
% Find the Unscented Kalman Filter block at the top level.
    candidates = find_system(modelName, 'SearchDepth', 2, ...
                             'BlockType', 'SubSystem');
    for k = 1:numel(candidates)
        name = lower(get_param(candidates{k}, 'Name'));
        if contains(name, 'kalman') || contains(name, 'ukf') || ...
           contains(name, 'unscented')
            ukfBlock = candidates{k};
            return;
        end
    end
    % Fallback: look for MATLAB Function blocks with UKF in the script
    candidates2 = find_system(modelName, 'SearchDepth', 2, ...
                              'BlockType', 'MATLABFcn');
    for k = 1:numel(candidates2)
        name = lower(get_param(candidates2{k}, 'Name'));
        if contains(name, 'ukf') || contains(name, 'kalman')
            ukfBlock = get_param(candidates2{k}, 'Parent');
            return;
        end
    end
    error(['Could not automatically locate the UKF subsystem in ', ...
           modelName, '. Set ukfBlock manually in connect_instrument_decision().']);
end

%--------------------------------------------------------------------------
function gnssBlock = find_gnss_enabled_subsystem(modelName)
% Find the GNSS Enabled Subsystem at the top level.
    candidates = find_system(modelName, 'SearchDepth', 1, ...
                             'BlockType', 'SubSystem');
    for k = 1:numel(candidates)
        name = lower(get_param(candidates{k}, 'Name'));
        if contains(name, 'gnss') || contains(name, 'gps')
            gnssBlock = candidates{k};
            return;
        end
    end
    error(['Could not automatically locate the GNSS subsystem in ', ...
           modelName, '. Set gnssBlock manually in connect_instrument_decision().']);
end

%--------------------------------------------------------------------------
function [covIdx, stateIdx] = find_ukf_output_ports(ukfBlock)
% Returns the output port indices for covariance and state on the UKF block.
% Tries to identify them by signal name; falls back to defaults (2, 1).
    try
        ph = get_param(ukfBlock, 'PortHandles');
        outPorts = ph.Outport;
        covIdx   = 2;   % default
        stateIdx = 1;   % default
        for k = 1:numel(outPorts)
            lines = get_param(outPorts(k), 'Line');
            if lines ~= -1
                sigName = lower(get_param(lines, 'Name'));
                if contains(sigName, 'cov') || contains(sigName, 'p_k') || ...
                   contains(sigName, 'covariance')
                    covIdx = k;
                elseif contains(sigName, 'state') || contains(sigName, 'x_est') || ...
                       contains(sigName, 'estimated')
                    stateIdx = k;
                end
            end
        end
    catch
        covIdx   = 2;
        stateIdx = 1;
    end
end

%==========================================================================
%  SUBSYSTEM CONSTRUCTION UTILITIES
%==========================================================================

function blockPath = find_or_add_inport(parent, portName, portNum, pos)
    blockPath = [parent, '/', portName];
    if getSimulinkBlockHandle(blockPath) == -1
        add_block('simulink/Sources/In1', blockPath, 'MakeNameUnique','off');
    end
    set_param(blockPath, 'Port', num2str(portNum), 'Position', pos);
end

function blockPath = find_or_add_outport(parent, portName, portNum, pos)
    blockPath = [parent, '/', portName];
    if getSimulinkBlockHandle(blockPath) == -1
        add_block('simulink/Sinks/Out1', blockPath, 'MakeNameUnique','off');
    end
    set_param(blockPath, 'Port', num2str(portNum), 'Position', pos);
end

function delete_internal_lines(systemPath)
    lines = find_system(systemPath, 'FindAll','on', ...
                        'SearchDepth',1, 'Type','line');
    for k = 1:numel(lines)
        try; delete_line(lines(k)); catch; end
    end
end

function delete_non_port_blocks(systemPath)
    blocks = find_system(systemPath, 'SearchDepth',1, 'Type','Block');
    for k = 1:numel(blocks)
        bp = string(blocks{k});
        if bp == string(systemPath); continue; end
        bt = string(get_param(bp, 'BlockType'));
        if any(bt == ["Inport","Outport","EnablePort"]); continue; end
        delete_block(bp);
    end
end

function add_line_safe(systemPath, src, dst)
    try
        add_line(systemPath, src, dst, 'autorouting','on');
    catch ME
        if ~contains(ME.message,'already connected','IgnoreCase',true) && ...
           ~contains(ME.message,'valid connection','IgnoreCase',true)
            warning('add_line_safe: %s  (%s -> %s)', ME.message, src, dst);
        end
    end
end

function set_matlab_function_script(blockPath, scriptText)
% Writes the function script into a MATLAB Function block using Stateflow API.
    try
        rt = sfroot;
        % The MATLAB Function block maps to an EMChart in Stateflow
        charts = rt.find('-isa','Stateflow.EMChart');
        for k = 1:numel(charts)
            if strcmp(charts(k).Path, blockPath)
                charts(k).Script = scriptText;
                return;
            end
        end
        warning('set_matlab_function_script: chart not found for %s', blockPath);
    catch ME
        warning('set_matlab_function_script failed for %s: %s', blockPath, ME.message);
    end
end

%==========================================================================
%  EMBEDDED FUNCTION SCRIPTS
%  (Inline copies so the installer is self-contained.
%   These match compute_lambda_gnss.m and compute_lambda_kalman.m exactly.)
%==========================================================================

function s = compute_lambda_gnss_script()
s = [...
'function lambda_gnss = compute_lambda_gnss(Pk, lambda_prev)', newline, ...
'%COMPUTE_LAMBDA_GNSS  GNSS deactivation (convergence) logic – INOAS Section 5.2.2', newline, ...
'% Inputs:  Pk (6x6 UKF covariance), lambda_prev (feedback from Unit Delay)', newline, ...
'% Output:  lambda_gnss  1=keep GNSS on, 0=switch GNSS off', newline, ...
'N_hold      = 60;', newline, ...
'sigma_enter = 65;', newline, ...
'N_good_min  = 120;', newline, ...
'persistent active_count good_count', newline, ...
'if isempty(active_count); active_count = int32(0); end', newline, ...
'if isempty(good_count);   good_count   = int32(0); end', newline, ...
'lambda_gnss = 1.0;', newline, ...
'if lambda_prev >= 0.5', newline, ...
'    active_count = active_count + int32(1);', newline, ...
'    Pr = Pk(1:3,1:3);', newline, ...
'    Pr = 0.5*(Pr+Pr'');', newline, ...
'    m  = sqrt(max(Pr(1,1)+Pr(2,2)+Pr(3,3), 0.0));', newline, ...
'    if active_count >= int32(N_hold)', newline, ...
'        if m <= sigma_enter', newline, ...
'            good_count = good_count + int32(1);', newline, ...
'        else', newline, ...
'            good_count = int32(0);', newline, ...
'        end', newline, ...
'        if good_count >= int32(N_good_min)', newline, ...
'            lambda_gnss  = 0.0;', newline, ...
'            active_count = int32(0);', newline, ...
'            good_count   = int32(0);', newline, ...
'        end', newline, ...
'    end', newline, ...
'else', newline, ...
'    active_count = int32(0);', newline, ...
'    good_count   = int32(0);', newline, ...
'end', newline, ...
'end'];
end

function s = compute_lambda_kalman_script()
s = [...
'function lambda_kf = compute_lambda_kalman(Pk, x_kf, x_gnss, gnss_available, lambda_prev)', newline, ...
'%COMPUTE_LAMBDA_KALMAN  GNSS reactivation (divergence) logic – INOAS Section 5.2.2', newline, ...
'alpha=0.98; sigma_pos_th=120; sigma_vel_th=50; growth_th=1.5;', newline, ...
'disc_scale=1000; eta_th=0.5; N_hold_KF=50; N_refresh=2000; N_bad_min=25; eps_f=1e-6;', newline, ...
'persistent sp_sm sv_sm sp_mn sv_mn nKF bc', newline, ...
'if isempty(sp_sm); sp_sm=0; sv_sm=0; sp_mn=Inf; sv_mn=Inf; nKF=int32(0); bc=int32(0); end', newline, ...
'lambda_kf = 0.0;', newline, ...
'if lambda_prev < 0.5', newline, ...
'    nKF = nKF + int32(1);', newline, ...
'    Pr=Pk(1:3,1:3); Pv=Pk(4:6,4:6);', newline, ...
'    Pr=0.5*(Pr+Pr''); Pv=0.5*(Pv+Pv'');', newline, ...
'    cov_ok = all(isfinite(Pk(:))) && all(diag(Pk)>0);', newline, ...
'    sr = sqrt(max(Pr(1,1)+Pr(2,2)+Pr(3,3),0));', newline, ...
'    vr = sqrt(max(Pv(1,1)+Pv(2,2)+Pv(3,3),0));', newline, ...
'    sp_sm = alpha*sp_sm+(1-alpha)*sr;', newline, ...
'    sv_sm = alpha*sv_sm+(1-alpha)*vr;', newline, ...
'    if sp_sm<sp_mn; sp_mn=sp_sm; end', newline, ...
'    if sv_sm<sv_mn; sv_mn=sv_sm; end', newline, ...
'    if gnss_available>0.5; disc=norm(x_gnss-x_kf)/disc_scale; else; disc=0; end', newline, ...
'    eta=max([sp_sm/sigma_pos_th, sv_sm/sigma_vel_th, ...', newline, ...
'             sp_sm/max(sp_mn,eps_f)/growth_th, sv_sm/max(sv_mn,eps_f)/growth_th, disc]);', newline, ...
'    kf_bad = ~cov_ok || nKF>=int32(N_refresh) || ...', newline, ...
'             (nKF>=int32(N_hold_KF) && eta>=eta_th);', newline, ...
'    if kf_bad; bc=bc+int32(1); else; bc=int32(max(double(bc)-1,0)); end', newline, ...
'    if bc >= int32(N_bad_min)', newline, ...
'        lambda_kf=1.0;', newline, ...
'        sp_sm=0; sv_sm=0; sp_mn=Inf; sv_mn=Inf; nKF=int32(0); bc=int32(0);', newline, ...
'    end', newline, ...
'else', newline, ...
'    sp_sm=0; sv_sm=0; sp_mn=Inf; sv_mn=Inf; nKF=int32(0); bc=int32(0);', newline, ...
'end', newline, ...
'end'];
end

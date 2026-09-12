function configure_navigation_prediction(model)
%CONFIGURE_NAVIGATION_PREDICTION Apply the reviewable Simulink wiring patch.
% Idempotent. Does not save the model; inspect it, then call save_system(model).
if nargin == 0
    model = 'inoas_model';
end
model = char(model);
if ~bdIsLoaded(model)
    rootDir = fileparts(fileparts(mfilename('fullpath')));
    load_system(fullfile(rootDir, 'models', [model '.slx']));
end

ukf = [model '/Kalman Filter/Unscented Kalman Filter_'];
set_param(ukf, 'Alpha','ukfAlpha', 'Beta','ukfBeta', 'Kappa','ukfKappa');

% The MPC state and covariance now refer to the same posterior estimate.
source = get_param([model '/Kalman Filter'], 'PortHandles');
dest = get_param([model '/Mux'], 'PortHandles');
connect(model, source.Outport(1), dest.Inport(1));
connect(model, source.Outport(2), dest.Inport(2));

chart = findChart([model '/Instrument Decision/Instrument Decision FSM']);
chart.Script = strjoin([
    "function [lambda, nav_status] = supervisor_block(NIS,n_sat,PDOP,HPE,VPE,gnss_sol)"
    "%#codegen"
    "[lambda,nav_status] = instrument_decision(NIS,n_sat,PDOP,HPE,VPE,gnss_sol, ..."
    "    gnssOnDuration,gnssOffDuration,pseudoNisThreshold,Ts);"
    "end"], newline);
for name = ["gnssOnDuration","gnssOffDuration","pseudoNisThreshold","Ts"]
    parameter(chart, name);
end
outputData(chart, 'nav_status', '[3 1]');
parent = [model '/Instrument Decision'];
ensureBlock('simulink/Sinks/Out1', [parent '/Navigation Status'], ...
    'Port','2', 'Position',[810 240 840 260]);
src = get_param(chart.Path, 'PortHandles');
dst = get_param([parent '/Navigation Status'], 'PortHandles');
connect(parent, src.Outport(2), dst.Inport(1));

% The status snapshot follows exactly the same delay as effective lambda.
ensureBlock('simulink/Discrete/Unit Delay', [model '/Navigation Status Delay'], ...
    'SampleTime','Ts', 'InitialCondition','[lamda_init;0;1]', ...
    'Position',[1080 530 1130 570]);
ensureBlock('simulink/Discrete/Zero-Order Hold', [model '/Navigation Status MPC Hold'], ...
    'SampleTime','h', 'Position',[1190 530 1240 570]);
src = get_param(parent, 'PortHandles');
delay = get_param([model '/Navigation Status Delay'], 'PortHandles');
hold = get_param([model '/Navigation Status MPC Hold'], 'PortHandles');
connect(model, src.Outport(2), delay.Inport(1));
connect(model, delay.Outport(1), hold.Inport(1));

chart = findChart([model '/MPC']);
chart.Script = strjoin([
    "function [u,delta_Ulast,slack_opt] = MPC_INOAS_block(x_estim,covariance_estim,t_sim,nav_status)"
    "%#codegen"
    "coder.extrinsic('MPC_INOAS');"
    "u = zeros(3,1);"
    "delta_Ulast = zeros(375,1);"
    "slack_opt = zeros(125,1);"
    "[u_tmp,delta_tmp,slack_tmp] = MPC_INOAS(x_estim,covariance_estim,t_sim,nav_status);"
    "u = u_tmp;"
    "delta_Ulast = delta_tmp;"
    "slack_opt = slack_tmp;"
    "end"], newline);
data = chart.find('-isa','Stateflow.Data','Name','nav_status');
if isempty(data)
    data = Stateflow.Data(chart);
    data.Name = 'nav_status';
end
data.Scope = 'Input';
data.Port = 4;
data.Props.Array.Size = '[3 1]';
dst = get_param(chart.Path, 'PortHandles');
connect(model, hold.Outport(1), dst.Inport(4));

chart = findChart([model '/Kalman Filter/MATLAB Function']);
chart.Script = strjoin([
    "function enabled = gatillo_gnss(lambda,t)"
    "%#codegen"
    "epoch = (t-gnssFixEpoch)/gnss_sample_time;"
    "enabled = lambda && abs(epoch-round(epoch)) < 1e-8;"
    "end"], newline);
parameter(chart, 'gnssFixEpoch');
parameter(chart, 'gnss_sample_time');

% Preserve the existing diagnostic semantics, but remove stale 100/50 m.
% This is a residual score using R, NOT an innovation test using H*P*H'+R.
chart = findChart([model '/Kalman Filter/Pseudo NIS']);
chart.Script = strjoin([
    "function [NIS,error_bruto] = compute_NIS(Z_gnss,Z_mag_alt,lamda,x_hat)"
    "%#codegen"
    "% Legacy pseudo-NIS. Threshold remains heuristic."
    "persistent Z_gnss_prev x_hat_held"
    "if isempty(Z_gnss_prev)"
    "    Z_gnss_prev = zeros(6,1); x_hat_held = zeros(6,1);"
    "end"
    "if sum(abs(Z_gnss(1:3)-Z_gnss_prev(1:3))) > 1000"
    "    x_hat_held = x_hat; Z_gnss_prev = Z_gnss;"
    "end"
    "error_bruto = zeros(6,1);"
    "if lamda"
    "    y = Z_gnss-x_hat_held;"
    "    NIS = y.'*(R_gnss\y);"
    "    error_bruto = y;"
    "else"
    "    y = Z_mag_alt-myMeasurementFcn(x_hat);"
    "    NIS = y.'*(R_matrix\y);"
    "    error_bruto(1:4) = y;"
    "end"
    "end"], newline);
parameter(chart, 'R_matrix');
parameter(chart, 'R_gnss');

set_param(model, 'SignalLogging','on', 'SignalLoggingName','logsout');
logPort([model '/Kalman Filter/Unscented Kalman Filter_'],1,'ukf_state');
logPort([model '/Kalman Filter/Unscented Kalman Filter_'],2,'ukf_covariance');
logPort([model '/Kalman Filter'],4,'NIS');
logPort([model '/Instrument Decision'],1,'lambda_command');
logPort([model '/Unit Delay1'],1,'lambda');
logPort([model '/Navigation Status Delay'],1,'navigation_status');
logPort([model '/Kalman Filter/MATLAB Function'],1,'gnss_update');
logPort([model '/Spacecraft Dynamics'],1,'truth_position_eci');
logPort([model '/Spacecraft Dynamics'],2,'truth_velocity_eci');
fprintf('Navigation prediction wiring configured in %s (not saved).\n', model);
end

function chart = findChart(path)
r = sfroot;
chart = r.find('-isa','Stateflow.EMChart','Path',path);
assert(numel(chart)==1, 'INOAS:ChartNotFound', 'Expected one chart at %s.', path);
end

function parameter(chart, name)
data = chart.find('-isa','Stateflow.Data','Name',char(name));
if isempty(data)
    data = Stateflow.Data(chart);
    data.Name = char(name);
end
data.Scope = 'Parameter';
end

function outputData(chart, name, dimensions)
data = chart.find('-isa','Stateflow.Data','Name',name);
if isempty(data)
    data = Stateflow.Data(chart);
    data.Name = name;
end
data.Scope = 'Output';
data.Port = 2;
data.Props.Array.Size = dimensions;
end

function ensureBlock(library, path, varargin)
if getSimulinkBlockHandle(path) == -1
    add_block(library, path, varargin{:});
end
end

function connect(system, source, dest)
line = get_param(dest,'Line');
if line ~= -1
    previous = get_param(line,'SrcPortHandle');
    if previous == source
        return;
    end
    delete_line(system, previous, dest);
end
add_line(system, source, dest, 'autorouting','on');
end

function logPort(path, index, name)
p = get_param(path,'PortHandles');
set_param(p.Outport(index), 'DataLogging','on', ...
    'DataLoggingNameMode','Custom', 'DataLoggingName',name);
end

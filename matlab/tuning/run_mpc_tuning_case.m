function result = run_mpc_tuning_case(modelName, weights, stopTime)
%RUN_MPC_TUNING_CASE Run one INOAS simulation with candidate MPC weights.
%
% The scenario is assumed to have been initialized already in the base
% workspace by initialize_inoas_simulation.m. Only Q_step, R_step and S_step
% (and their expanded horizon matrices Q, R and S) are changed here.

arguments
    modelName (1,1) string
    weights struct
    stopTime (1,1) double {mustBePositive}
end

q = reshape(double(weights.Q_step), 1, []);
r = reshape(double(weights.R_step), 1, []);
s = reshape(double(weights.S_step), 1, []);

if numel(q) ~= 6 || numel(r) ~= 3 || numel(s) ~= 3
    error('Expected Q_step(1x6), R_step(1x3), S_step(1x3).');
end
if any(~isfinite([q r s])) || any([q r s] <= 0)
    error('All MPC tuning weights must be finite and strictly positive.');
end

Np = evalin('base', 'Np');
assignin('base', 'Q_step', q);
assignin('base', 'R_step', r);
assignin('base', 'S_step', s);
assignin('base', 'Q', diag(repmat(q, 1, Np)));
assignin('base', 'R', diag(repmat(r, 1, Np)));
assignin('base', 'S', diag(repmat(s, 1, Np)));

% Remove logs produced by the previous candidate. MPC_INOAS resets its
% persistent state when simulation time returns to zero.
logVars = [ ...
    "mpc_dsafe_log_time", "mpc_dsafe_log_first", "mpc_dsafe_log_max", ...
    "mpc_dsafe_snapshot_time", "mpc_dsafe_snapshot_profile", ...
    "mpc_dsafe_snapshot_distance", "mpc_dsafe_snapshot_margin", ...
    "mpc_dsafe_snapshot_slack", "t_CA_log", "sigma_CA_log", "dsafe_CA_log"];
for k = 1:numel(logVars)
    evalin('base', sprintf('clear %s', logVars(k)));
end

try
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('StopTime', num2str(stopTime, '%.15g'));
    out = sim(simIn);
    metrics = extract_mpc_tuning_metrics(out, stopTime);
    ok = true;
    message = "";
catch ME
    metrics = emptyMetrics(stopTime);
    ok = false;
    message = string(getReport(ME, 'extended', 'hyperlinks', 'off'));
end

result = struct();
result.ok = ok;
result.message = message;
result.stopTime = stopTime;
result.Q_step = q;
result.R_step = r;
result.S_step = s;
result.metrics = metrics;
end

function m = emptyMetrics(stopTime)
m = struct( ...
    'deltaV', inf, ...
    'minSeparation', -inf, ...
    'minRobustMargin', -inf, ...
    'finalPositionError', inf, ...
    'postEncounterRmsError', inf, ...
    'maxPositionError', inf, ...
    'maxControlAbs', inf, ...
    'controlLimit', nan, ...
    'finiteSignals', false, ...
    'stopTime', stopTime);
end

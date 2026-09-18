function [P_nodes, P_all] = predictNavigationCovarianceProfile( ...
    x0, P0, t0, targetTimes, uNominal, cfg)

dt = cfg.sampleTime;

steps = (targetTimes(:) - t0) / dt;
assert(all(steps >= 1) && all(abs(steps-round(steps)) < 1e-8), ...
    'MPC prediction times must match UKF time steps.');
steps = round(steps);

%% Dynamic Q associated with commanded acceleration

uNominal = uNominal(:);

% 10% actuator execution uncertainty per commanded axis
Sigma_act = diag((cfg.errorCmd .* uNominal).^2);

% Propagate acceleration uncertainty into [position; velocity]
Q_act = cfg.Gcmd * Sigma_act * cfg.Gcmd';

% Total process covariance
Qk = cfg.Qexternal + Q_act;

persistent qlog_t qlog_ext qlog_act qlog_total

if isempty(qlog_t)
    qlog_t = [];
    qlog_ext = [];
    qlog_act = [];
    qlog_total = [];
end

qlog_t(end+1,1) = t0;
qlog_ext(end+1,1) = max(abs(cfg.Qexternal(:)));
qlog_act(end+1,1) = max(abs(Q_act(:)));
qlog_total(end+1,1) = max(abs(Qk(:)));

assignin('base','qlog_t',qlog_t);
assignin('base','qlog_Qext_max',qlog_ext);
assignin('base','qlog_Qact_max',qlog_act);
assignin('base','qlog_Qtotal_max',qlog_total);

% Numerical symmetry
Qk = 0.5 * (Qk + Qk.');

%% Navigation UKF used for covariance forecast

filter = unscentedKalmanFilter( ...
    cfg.stateTransitionFcn, cfg.measurementFcn, x0, ...
    'StateCovariance', P0, ...
    'ProcessNoise', Qk, ...
    'MeasurementNoise', cfg.Raux, ...
    'Alpha', cfg.alpha, ...
    'Beta', cfg.beta, ...
    'Kappa', cfg.kappa);

P_all = zeros(6,6,steps(end));

for k = 1:steps(end)

    predict(filter, uNominal);

    innovationAtZero = residual(filter, zeros(4,1));
    correct(filter, -innovationAtZero);

    P_all(:,:,k) = 0.5 * ...
        (filter.StateCovariance + filter.StateCovariance.');

end

P_nodes = P_all(:,:,steps);

end
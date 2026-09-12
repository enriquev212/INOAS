function [P_nodes, forecast] = predictNavigationCovarianceProfile( ...
    x0, P0, t0, targetTimes, uNominal, navStatus, cfg)
%PREDICTNAVIGATIONCOVARIANCEPROFILE Nominal posterior UKF covariance forecast.
% Starts AFTER current measurements. Future auxiliary corrections use their
% expected measurement (zero innovation), not future data or plant truth.
% cfg.mode: aux_only, scheduled, or no_measurements (verification reference).
% navStatus: effective [lambda; elapsed seconds; healthy] after Unit Delay.
% The nominal ECI acceleration is held fixed during the forecast. This is a
% forecast of estimation error, not a guarantee on physical tracking error.

validateattributes(x0, {'double'}, {'column','numel',6,'finite'});
validateattributes(uNominal, {'double'}, {'column','numel',3,'finite'});
validateattributes(t0, {'double'}, {'scalar','finite'});
validateattributes(cfg.sampleTime, {'double'}, {'scalar','positive','finite'});
mode = validatestring(cfg.mode, {'aux_only','scheduled','no_measurements'});
dt = cfg.sampleTime;
targetTimes = targetTimes(:);
steps = (targetTimes - t0) / dt;
assert(~isempty(steps) && all(isfinite(steps)) && all(steps >= 1) && ...
    all(diff(steps) > 0) && all(abs(steps-round(steps)) < 1e-7), ...
    'INOAS:NavigationGrid', 'Prediction times must be future ticks of the UKF clock.');
steps = round(steps);
if strcmp(func2str(cfg.stateTransitionFcn), 'myStateTransitionFcn')
    assert(abs(dt-1) < 1e-12, 'INOAS:EstimatorStep', ...
        'myStateTransitionFcn propagates 1 s; Ts must remain 1 s.');
end
P0 = checkedCovariance(P0, 6, 'P0');
Q = checkedCovariance(cfg.Q, 6, 'Q');
Raux = checkedCovariance(cfg.Raux, 4, 'Raux');
Rgnss = checkedCovariance(cfg.Rgnss, 6, 'Rgnss');

lambda = false;
elapsed = 0;
healthy = false;
if strcmp(mode, 'scheduled')
    validateattributes(navStatus, {'double'}, {'vector','numel',3,'finite'});
    assert(ismember(navStatus(1), [0,1]) && navStatus(2) >= 0 && ...
        ismember(navStatus(3), [0,1]), 'INOAS:NavigationStatus', ...
        'Expected effective [lambda; elapsed seconds; healthy].');
    validateattributes(cfg.gnssSampleTime, {'double'}, {'scalar','positive','finite'});
    ratio = cfg.gnssSampleTime / dt;
    assert(abs(ratio-round(ratio)) < 1e-10, 'INOAS:GnssGrid', ...
        'The GNSS period must be an integer multiple of Ts.');
    lambda = logical(navStatus(1));
    elapsed = navStatus(2);
    healthy = logical(navStatus(3));
end

filter = unscentedKalmanFilter(cfg.stateTransitionFcn, cfg.measurementFcn, x0, ...
    'StateCovariance', P0, 'ProcessNoise', Q, 'MeasurementNoise', Raux, ...
    'Alpha', cfg.alpha, 'Beta', cfg.beta, 'Kappa', cfg.kappa);
nSteps = steps(end);
forecast.time = t0 + (1:nSteps).' * dt;
forecast.P_prior = zeros(6,6,nSteps);
forecast.P_posterior = zeros(6,6,nSteps);
forecast.gnss_update = false(nSteps,1);
forecast.lambda = false(nSteps,1);
forecast.mode = mode;
forecast.nominal_input_eci = uNominal;

for k = 1:nSteps
    [~, Pprior] = predict(filter, uNominal);
    forecast.P_prior(:,:,k) = Pprior;
    if ~strcmp(mode, 'no_measurements')
        % residual(0) = -predicted measurement mean, including nonlinear terms.
        innovationAtZero = residual(filter, zeros(4,1));
        correct(filter, -innovationAtZero);
    end
    if strcmp(mode, 'scheduled')
        % No future NIS triggers or future dataset health are inspected.
        [lambda, elapsed] = navigationDutyCycleStep(lambda, elapsed, healthy, ...
            0, cfg.onDuration, cfg.offDuration, cfg.scoreThreshold, dt);
        epoch = (forecast.time(k)-cfg.gnssEpoch) / cfg.gnssSampleTime;
        freshFix = abs(epoch-round(epoch)) < 1e-8;
        forecast.lambda(k) = lambda;
        if lambda && healthy && freshFix
            % The actual GNSS measurement function is H = I_6. A nominal
            % zero-innovation correction leaves the mean unchanged.
            P = filter.StateCovariance;
            K = P / (P + Rgnss);
            E = eye(6) - K;
            P = E*P*E.' + K*Rgnss*K.';
            filter.StateCovariance = (P+P.')/2;
            forecast.gnss_update(k) = true;
        end
    end
    P = filter.StateCovariance;
    forecast.P_posterior(:,:,k) = (P+P.')/2;
end
P_nodes = forecast.P_posterior(:,:,steps);
forecast.node_steps = steps;
end

function P = checkedCovariance(P, n, label)
validateattributes(P, {'double'}, {'size',[n,n],'finite','real'}, mfilename, label);
assert(norm(P-P.','fro') <= 1e-10*max(1,norm(P,'fro')), ...
    'INOAS:CovarianceSymmetry', '%s must be symmetric.', label);
P = (P+P.')/2;
assert(min(eig(P)) >= -1e-10*max(1,norm(P,2)), ...
    'INOAS:CovariancePSD', '%s must be positive semidefinite.', label);
end

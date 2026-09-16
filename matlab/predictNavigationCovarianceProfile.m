function P_nodes = predictNavigationCovarianceProfile( ...
    x0, P0, t0, targetTimes, uNominal, cfg)

    dt = cfg.sampleTime;
    
    steps = (targetTimes(:) - t0) / dt;
    assert(all(steps >= 1) && all(abs(steps-round(steps)) < 1e-8), ...
        'MPC prediction times must match UKF time steps.');
    steps = round(steps);
    
    filter = unscentedKalmanFilter( ...
        cfg.stateTransitionFcn, cfg.measurementFcn, x0, ...
        'StateCovariance', P0, ...
        'ProcessNoise', cfg.Q, ...
        'MeasurementNoise', cfg.Raux, ...
        'Alpha', cfg.alpha, ...
        'Beta', cfg.beta, ...
        'Kappa', cfg.kappa);
    
    P_all = zeros(6,6,steps(end));
    
    for k = 1:steps(end)
    
        predict(filter, uNominal);
    
        innovationAtZero = residual(filter, zeros(4,1));
        correct(filter, -innovationAtZero);
    
        P_all(:,:,k) = 0.5*(filter.StateCovariance + filter.StateCovariance.');
    end
    
    P_nodes = P_all(:,:,steps);

end
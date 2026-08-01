function [u, delta_Ulast, slack_opt] = MPC_INOAS(x_estim, covariance_estim, t_sim, varargin)
%MPC_INOAS MPC guidance law for nominal tracking and debris avoidance.
%
% Inputs:
%   x_estim          Estimated absolute spacecraft state in ECI [6x1].
%   covariance_estim Estimated state covariance, either 6x6, 3x3, or vectorized.
%   t_sim            Current simulation time [s]. If omitted, an internal clock is
%                    advanced using the model sample time.
%
% Outputs:
%   u                Commanded absolute acceleration in ECI [3x1].
%   delta_Ulast      Last optimized delta-control sequence, reused as warm start.
%   slack_opt        Debris-avoidance slack variables over the prediction horizon.

    %% Configuration and persistent controller state
    cfg = getMpcConfig(varargin{:});
    hasExternalTime = (nargin >= 3) && ~isempty(t_sim);

    x_estim = x_estim(:);

    nx = cfg.nx;
    m  = cfg.m;
    Np = cfg.Np;
    h  = cfg.h;
    sampleTime = cfg.sampleTime;

    Q = cfg.Q;
    R = cfg.R;
    S = cfg.S;

    Umin = cfg.Umin;
    Umax = cfg.Umax;
    Ymin = cfg.Ymin;
    Ymax = cfg.Ymax;
    deltaUmax = cfg.deltaUmax;
    slackWeight = cfg.slackWeight;

    dsafe0 = cfg.dsafe0;
    safetyCost = cfg.safetyCost;
    rk_debris = cfg.rk_debris(:);
    x_debris_hist = cfg.x_debris_hist;
    Q_cov = cfg.Q_cov;
    covarianceFrame = cfg.covarianceFrame;
    covarianceMetric = cfg.covarianceMetric;
    logDsafe = cfg.logDsafe;
    dsafeSnapshotTimes = cfg.dsafeSnapshotTimes(:);

    A_c = cfg.A_c;
    B_c = cfg.B_c;

    r_p_full = cfg.r_p_full;
    Ntimesteps = cfg.Ntimesteps;

    persistent u_abs_current delta_Ulast_internal last_solved_step last_slack_internal
    persistent dsafe_log_time dsafe_log_first dsafe_log_max
    persistent dsafe_snapshot_time_log dsafe_snapshot_profile_log
    persistent dsafe_snapshot_distance_log dsafe_snapshot_margin_log dsafe_snapshot_slack_log
    persistent t_internal

    if isempty(u_abs_current)
        u_abs_current = cfg.u0(:);
    end

    if isempty(delta_Ulast_internal)
        delta_Ulast_internal = zeros(m*Np,1);
    end

    if ~hasExternalTime
        if isempty(t_internal)
            t_sim = 0;
        else
            t_sim = t_internal;
        end
    else
        t_sim = double(t_sim);
    end

    if t_sim <= 1e-9
        u_abs_current = cfg.u0(:);
        delta_Ulast_internal = zeros(m*Np,1);
        last_solved_step = [];
        last_slack_internal = zeros(Np,1);
        dsafe_log_time = [];
        dsafe_log_first = [];
        dsafe_log_max = [];
        dsafe_snapshot_time_log = [];
        dsafe_snapshot_profile_log = [];
        dsafe_snapshot_distance_log = [];
        dsafe_snapshot_margin_log = [];
        dsafe_snapshot_slack_log = [];
        t_internal = 0;
    end

    if numel(delta_Ulast_internal) ~= m*Np
        delta_Ulast_internal = zeros(m*Np,1);
    end

    timeStep = floor(t_sim/h) + 1;
    timeStep = max(1, min(timeStep, Ntimesteps));

    delta_Ulast = delta_Ulast_internal;

    if ~isempty(last_solved_step) && last_solved_step == timeStep
        u = u_abs_current;
        delta_Ulast = padColumnVector(delta_Ulast_internal, cfg.m * cfg.outputNp);
        slack_opt = padColumnVector(last_slack_internal, cfg.outputNp);

        if ~hasExternalTime
            t_internal = t_sim + sampleTime;
        else
            t_internal = t_sim;
        end
        return;
    end

    %% Build prediction-horizon reference
    r_p = zeros(nx*Np,1);

    for i = 1:Np
        refStep = timeStep + i - 1;
        refStep = min(refStep, Ntimesteps);

        idx_mpc = (i-1)*nx + (1:nx);
        idx_ref = (refStep-1)*nx + (1:nx);

        r_p(idx_mpc) = r_p_full(idx_ref);
    end

    %% Transform current absolute state into the reference LVLH frame
    idx_now = (timeStep-1)*nx + (1:nx);
    x_ref_now_abs = r_p_full(idx_now);
    x_ref_now_abs = x_ref_now_abs(:);

    r_abs = x_estim(1:3);
    v_abs = x_estim(4:6);

    r_ref = x_ref_now_abs(1:3);
    v_ref = x_ref_now_abs(4:6);

    dr_abs = r_abs - r_ref;
    dv_abs = v_abs - v_ref;

    [T_abs_to_ref, T_ref_to_abs] = referenceFrameTransform(r_ref, v_ref);

    u_ff_abs = getReferenceFeedforward(cfg, t_sim);
    u_ff_ref = T_abs_to_ref * u_ff_abs;
    ff_horizon_ref = repmat(u_ff_ref, Np, 1);

    if cfg.useReferenceFeedforward
        u_current = T_abs_to_ref * u_abs_current - u_ff_ref;
    else
        u_current = T_abs_to_ref * u_abs_current;
        ff_horizon_ref(:) = 0;
    end

    u = u_current;

    dr_ref = T_abs_to_ref * dr_abs;

    dv_ref_inertial = T_abs_to_ref * dv_abs;

    h_ref_vec = cross(r_ref, v_ref);
    n_ref = norm(h_ref_vec) / norm(r_ref)^2;

    omega_lvlh = [0; 0; n_ref];

    dv_ref = dv_ref_inertial - cross(omega_lvlh, dr_ref);

    x_rel_estim = [dr_ref; dv_ref];

    covariance_estim = coerceCovarianceMatrix(covariance_estim, nx);

    if strcmpi(covarianceFrame, 'eci')
        T_cov = [T_abs_to_ref, zeros(3);
                 -skewSymmetric(omega_lvlh) * T_abs_to_ref, T_abs_to_ref];
        P_mpc = T_cov * covariance_estim * T_cov.';
    else
        P_mpc = covariance_estim;
    end

    P_mpc = symmetrizeCovariance(P_mpc);


    %% Discretize the linear relative dynamics
    Phi = expm(A_c*h);
    Gamma_hat = integral_gammahat(A_c,h);
    Gamma = Gamma_hat * B_c;

    %% Build free-response prediction matrices
    Phi_extend = zeros(Np*nx, nx);
    for i = 1:Np
        rows = (i-1)*nx + (1:nx);
        Phi_extend(rows,:) = Phi^i;
    end

    %% Build delta-control accumulator
    E = repmat(eye(m), Np, 1);

    H = zeros(Np*m, Np*m);
    for i = 1:Np
        for j = 1:i
            rows = (i-1)*m + (1:m);
            cols = (j-1)*m + (1:m);
            H(rows,cols) = eye(m);
        end
    end

    %% Build forced-response prediction matrices
    Gamma_u = zeros(Np*nx, Np*m);

    for i = 1:Np
        for j = 1:i
            rows = (i-1)*nx + (1:nx);
            cols = (j-1)*m  + (1:m);

            Gamma_u(rows,cols) = Phi^(i-j) * Gamma;
        end
    end

    Gamma_extend = Gamma_u * H;

    Y0 = Phi_extend*x_rel_estim + Gamma_u*(E*u);

    if ~cfg.quiet
        fprintf('t=%.2f | norm x_rel = %.3e | norm Y0 pos first = %.3e\n', ...
            t_sim, norm(x_rel_estim), norm(Y0(1:3)));
    end

    %% Warm-start the optimization
    Ndu = m*Np;
    Nslack = Np;
    
    delta_U0 = zeros(Ndu,1);
    
    for i = 1:Np-1
        idx_now_du  = (i-1)*m + (1:m);
        idx_next_du = i*m + (1:m);
        delta_U0(idx_now_du) = delta_Ulast(idx_next_du);
    end

    %% Select constrained state channels
    stateConstraintIdx = cfg.stateConstraintIdx;
    ny = numel(stateConstraintIdx);

    if ny > 0
        Gamma_extend_select = zeros(ny*Np, m*Np);
        Y0_select = zeros(ny*Np,1);

        for i = 1:Np
            row_sel = (i-1)*ny + (1:ny);
            row_x   = (i-1)*nx + stateConstraintIdx(:)';

            Gamma_extend_select(row_sel,:) = Gamma_extend(row_x,:);
            Y0_select(row_sel) = Y0(row_x);
        end
    else
        Gamma_extend_select = [];
        Y0_select = [];
    end

    %% Linearized debris-avoidance constraints
    LeftHandDebris = zeros(Np, m*Np + Np);
    RightHandDebris = zeros(Np,1);

    if dsafe0 > 0
        dsafe_profile = zeros(Np,1);
        P_k = P_mpc;
        for i = 1:Np
            P_k = Phi * P_k * Phi.' + Q_cov;
            P_k = symmetrizeCovariance(P_k);

            dsafe_k = dsafe0 + safetyCost * covarianceRadiusFromPosition(P_k(1:3,1:3), covarianceMetric);
            dsafe_profile(i) = dsafe_k;

            idx_state_i = (i-1)*nx + (1:nx);
            idx_pos     = (i-1)*nx + (1:3);
            refStep = min(timeStep + i - 1, Ntimesteps);

            x_ref_i = r_p(idx_state_i);
            r_ref_i = x_ref_i(1:3);
            r_debris_i = getDebrisPositionAtStep(x_debris_hist, refStep, nx, rk_debris);

            d_nom = T_abs_to_ref * (r_ref_i - r_debris_i);

            Gamma_k = Gamma_extend(idx_pos,:);
            Y0_k = Y0(idx_pos);

            scaleDebris = max([norm(d_nom)^2, dsafe_k^2, 1]);

            LeftHandDebris(i,1:m*Np) = (-2*d_nom' * Gamma_k) / scaleDebris;
            LeftHandDebris(i,m*Np+i) = -1 / scaleDebris;

            RightHandDebris(i) = (norm(d_nom)^2 - dsafe_k^2 + ...
                2*d_nom'*Y0_k) / scaleDebris;
        end

        if logDsafe
            dsafe_log_time(end+1,1) = t_sim;
            dsafe_log_first(end+1,1) = dsafe_profile(1);
            dsafe_log_max(end+1,1) = max(dsafe_profile);

            assignin('base', 'mpc_dsafe_log_time', dsafe_log_time);
            assignin('base', 'mpc_dsafe_log_first', dsafe_log_first);
            assignin('base', 'mpc_dsafe_log_max', dsafe_log_max);
        end

    end

    %% Decision-variable bounds
    lb_deltaU = -inf(m*Np,1);
    ub_deltaU =  inf(m*Np,1);

    lb_slack = zeros(Np,1);
    ub_slack = inf(Np,1);

    lb = [lb_deltaU; lb_slack];
    ub = [ub_deltaU; ub_slack];

    %% Assemble actuator, state, delta-control, and debris constraints
    Umin_eff = Umin;
    Umax_eff = Umax;
    if cfg.useReferenceFeedforward
        if ~isempty(Umin_eff)
            Umin_eff = Umin_eff - ff_horizon_ref;
        end
        if ~isempty(Umax_eff)
            Umax_eff = Umax_eff - ff_horizon_ref;
        end
    end

    [A,b] = MPCLinearConstraints(Umin_eff, Umax_eff, Ymin, Ymax, deltaUmax, ...
        u, E, H, Gamma_extend_select, Y0_select, ...
        LeftHandDebris, RightHandDebris, m, Np, dsafe0);

    %% Scale decision variables for numerical conditioning
    du_scale = deltaUmax(:);
    
    if isempty(du_scale)
        du_scale = 0.005*ones(Ndu,1);
    end
    
    du_scale(du_scale <= 0) = 0.005;
    
    Ddu = diag(du_scale);
    
    w0 = Ddu \ delta_U0;
    
    slack0 = zeros(Nslack,1);
    z0 = [w0; slack0];
    
    lb_w = lb_deltaU ./ du_scale;
    ub_w = ub_deltaU ./ du_scale;
    
    lb = [lb_w; lb_slack];
    ub = [ub_w; ub_slack];
    
    A_scaled = A;
    A_scaled(:,1:Ndu) = A(:,1:Ndu) * Ddu;

    %% Solve constrained MPC problem
    fun = @(z) MPCObjectiveScaled(Y0, Gamma_extend, Q, R, z, H, u, S, ...
                                  slackWeight, m, Np, Ddu);
    
    options = optimoptions('fmincon', ...
        'Display','none', ...
        'Algorithm','sqp', ...
        'SpecifyObjectiveGradient',true, ...
        'MaxIterations',50, ...
        'MaxFunctionEvaluations',2000, ...
        'ConstraintTolerance',1e-5, ...
        'OptimalityTolerance',1e-2, ...
        'StepTolerance',1e-5);
    
    [z_opt,fval,exitflag,output] = fmincon(fun,z0,A_scaled,b,[],[],lb,ub,[],options);
    
    w_opt = z_opt(1:Ndu);
    delta_U = Ddu*w_opt;

    z_unscaled = [delta_U; z_opt(Ndu+1:end)];
    
    viol = max(A*z_unscaled - b);
    
    delta_U_check = delta_U;
    U_check = ff_horizon_ref + E*u + H*delta_U_check;

    if ~cfg.quiet && mod(round(t_sim),10) == 0
        fprintf('t=%.1f | exit=%d | fval=%.3e | viol=%.1e | err=%.3f m\n', ...
            t_sim, exitflag, fval, viol, norm(x_rel_estim(1:3)));
    end
    %% Apply first optimized control move
    delta_u = delta_U(1:m);
    
    u_correction_ref = u_current + delta_u;
    u_ref = u_ff_ref + u_correction_ref;
    
    slack_opt_internal = z_opt(Ndu+1:end);

    if dsafe0 > 0 && ~isempty(dsafeSnapshotTimes)
        matchIdx = find(abs(dsafeSnapshotTimes - t_sim) <= 1e-9, 1, 'first');
        alreadyLoggedPred = any(abs(dsafe_snapshot_time_log - t_sim) <= 1e-9);

        if ~isempty(matchIdx) && ~alreadyLoggedPred
            Y_pred = Y0 + Gamma_extend * delta_U;
            distance_profile = zeros(Np,1);
            margin_profile = zeros(Np,1);

            for i = 1:Np
                idx_state_i = (i-1)*nx + (1:nx);
                idx_pos = (i-1)*nx + (1:3);
                refStep = min(timeStep + i - 1, Ntimesteps);

                x_ref_i = r_p(idx_state_i);
                r_ref_i = x_ref_i(1:3);
                r_debris_i = getDebrisPositionAtStep(x_debris_hist, refStep, nx, rk_debris);
                d_nom = T_abs_to_ref * (r_ref_i - r_debris_i);

                rel_vec_to_debris = Y_pred(idx_pos) + d_nom;
                distance_profile(i) = norm(rel_vec_to_debris);
                margin_profile(i) = distance_profile(i) - dsafe_profile(i);
            end

            dsafe_snapshot_time_log(end+1,1) = t_sim;
            dsafe_snapshot_profile_log(:,end+1) = dsafe_profile;
            dsafe_snapshot_distance_log(:,end+1) = distance_profile;
            dsafe_snapshot_margin_log(:,end+1) = margin_profile;
            dsafe_snapshot_slack_log(:,end+1) = slack_opt_internal(:);

            assignin('base', 'mpc_dsafe_snapshot_time', dsafe_snapshot_time_log);
            assignin('base', 'mpc_dsafe_snapshot_profile', dsafe_snapshot_profile_log);
            assignin('base', 'mpc_dsafe_snapshot_distance', dsafe_snapshot_distance_log);
            assignin('base', 'mpc_dsafe_snapshot_margin', dsafe_snapshot_margin_log);
            assignin('base', 'mpc_dsafe_snapshot_slack', dsafe_snapshot_slack_log);
        end
    end

    delta_Ulast_internal = delta_U;
    last_solved_step = timeStep;
    last_slack_internal = slack_opt_internal;
    delta_Ulast = padColumnVector(delta_Ulast_internal, cfg.m * cfg.outputNp);
    slack_opt = padColumnVector(slack_opt_internal, cfg.outputNp);

    u_abs = T_ref_to_abs * u_ref;

    u_abs_max = cfg.Umax(1);
    u_abs = max(min(u_abs, u_abs_max), -u_abs_max);

    u_abs_current = u_abs;
    u = u_abs;

    if ~hasExternalTime
        t_internal = t_sim + sampleTime;
    else
        t_internal = t_sim;
    end

end



%% Configuration helpers

function padded = padColumnVector(values, targetLength)
    values = values(:);
    targetLength = max(0, round(double(targetLength)));
    padded = zeros(targetLength, 1);

    nCopy = min(numel(values), targetLength);
    if nCopy > 0
        padded(1:nCopy) = values(1:nCopy);
    end
end

function cfg = getMpcConfig(varargin)
    cfg.nx = getBaseWorkspaceVar('nx', 6);
    cfg.m  = getBaseWorkspaceVar('m', 3);
    n = cfg.nx;
    m = cfg.m;

    cfg.Np = getBaseWorkspaceVar('Np', 40);
    cfg.outputNp = getBaseWorkspaceVar('mpcOutputNpMax', cfg.Np);
    cfg.outputNp = max(cfg.outputNp, cfg.Np);
    cfg.h  = getBaseWorkspaceVar('h', 1);
    cfg.sampleTime = getBaseWorkspaceVar('Ts', cfg.h);

    cfg.dsafe0 = getBaseWorkspaceVar('dsafe0', 1000);
    cfg.rk_debris = getBaseWorkspaceVar('rk_debris');
    cfg.x_debris_hist = coerceStateHistory(getBaseWorkspaceVar('x_debris_hist', []), n);


    cfg.u0 = getBaseWorkspaceVar('u', zeros(m, 1));    

    q_default = [10 * ones(1, min(3, n)), 0.1 * ones(1, max(n - 3, 0))];
    r_default = 10 * ones(1, m);
    cfg.Q = resolveWeightMatrix('Q', 'Q_step', q_default, n, cfg.Np);
    cfg.R = resolveWeightMatrix('R', 'R_step', r_default, m, cfg.Np);
    cfg.S = resolveWeightMatrix('S', 'S_step', r_default, m, cfg.Np);

    cfg.A_c = getBaseWorkspaceVar('A_c');
    cfg.B_c = getBaseWorkspaceVar('B_c');

    cfg.Umin = resolveBoundVector('Umin', 'U_min', m, cfg.Np, []);
    cfg.Umax = resolveBoundVector('Umax', 'U_max', m, cfg.Np, []);
    cfg.Ymin = resolveBoundVector('Ymin', 'Y_min', [], cfg.Np, []);
    cfg.Ymax = resolveBoundVector('Ymax', 'Y_max', [], cfg.Np, []);
    cfg.deltaUmax = resolveBoundVector('deltaUmax', 'deltaU_max', m, cfg.Np, []);
    cfg.slackWeight = getBaseWorkspaceVar('slackWeight', 1e6);
    cfg.safetyCost = getBaseWorkspaceVar('safetyCost', 0);
    cfg.Q_cov = resolveCovarianceMatrix({'Q_cov_mpc', 'Q_process_mpc', 'Q_covariance_mpc'}, n, zeros(n));
    cfg.covarianceFrame = getBaseWorkspaceVar('covarianceFrameMpc', 'eci');
    cfg.covarianceMetric = getBaseWorkspaceVar('covarianceMetricMpc', 'sqrt_trace_pos');
    cfg.logDsafe = getBaseWorkspaceVar('logDsafeMpc', false);
    cfg.dsafeSnapshotTimes = getBaseWorkspaceVar('dsafeSnapshotTimesMpc', []);
    cfg.useReferenceFeedforward = getBaseWorkspaceVar('useReferenceFeedforward', false);
    cfg.u_ff_ref_hist = getBaseWorkspaceVar('u_ff_ref_hist', []);
    cfg.t_ref = getBaseWorkspaceVar('t_ref', []);

    cfg.stateConstraintIdx = resolveStateConstraintIdx(cfg.Ymin, cfg.Ymax, n, cfg.Np);

    cfg.r_p_full = getBaseWorkspaceVar('r_p_full');
    cfg.Ntimesteps = getBaseWorkspaceVar('Ntimesteps');
    cfg.quiet = getBaseWorkspaceVar('mpcQuiet', false);
end

function u_ff_abs = getReferenceFeedforward(cfg, t_sim)
    u_ff_abs = zeros(cfg.m, 1);

    if ~cfg.useReferenceFeedforward || isempty(cfg.u_ff_ref_hist) || isempty(cfg.t_ref)
        return;
    end

    hist = cfg.u_ff_ref_hist;
    if size(hist, 1) ~= cfg.m && size(hist, 2) == cfg.m
        hist = hist.';
    end

    if size(hist, 1) ~= cfg.m
        return;
    end

    t = cfg.t_ref(:);
    if isempty(t) || size(hist, 2) ~= numel(t)
        return;
    end

    tq = min(max(double(t_sim), t(1)), t(end));
    u_ff_abs = interp1(t, hist.', tq, 'linear', 'extrap').';
    u_ff_abs = u_ff_abs(:);

    if numel(u_ff_abs) ~= cfg.m || any(~isfinite(u_ff_abs))
        u_ff_abs = zeros(cfg.m, 1);
    end
end


%% Optimization model

function [J, grad] = MPCObjectiveScaled(Y0, Gamma_extend, Q, R, z, H, u, S, ...
                                        slackWeight, m, Np, Ddu)

    Ndu = m*Np;

    w = z(1:Ndu);
    slack = z(Ndu+1:end);

    delta_U = Ddu*w;

    Y = Y0 + Gamma_extend*delta_U;

    U0 = repmat(u, Np, 1);
    U  = U0 + H*delta_U;

    J = 0.5*Y'*Q*Y + ...
        0.5*delta_U'*S*delta_U + ...
        0.5*U'*R*U + ...
        slackWeight*(slack'*slack);

    grad_deltaU = Gamma_extend.'*Q*Y + ...
                  S*delta_U + ...
                  H.'*R*U;

    grad_w = Ddu.'*grad_deltaU;

    grad_slack = 2*slackWeight*slack;

    grad = [grad_w; grad_slack];

end

function [A,b] = MPCLinearConstraints(Umin, Umax, Ymin, Ymax, deltaUmax, ...
    u, E, H, Gamma_extend_select, Y0_select, ...
    LeftHandDebris, RightHandDebris, m, Np, dsafe0)

    Ndu = m*Np;
    Nslack = Np;

    A_blocks = {};
    b_blocks = {};

    if ~isempty(Umin)
        A_blocks{end+1} = [-H, zeros(Ndu,Nslack)];
        b_blocks{end+1} = -Umin + E*u;
    end

    if ~isempty(Umax)
        A_blocks{end+1} = [H, zeros(Ndu,Nslack)];
        b_blocks{end+1} = Umax - E*u;
    end

    nyNp = size(Gamma_extend_select,1);

    if ~isempty(Ymin)
        A_blocks{end+1} = [-Gamma_extend_select, zeros(nyNp,Nslack)];
        b_blocks{end+1} = -Ymin + Y0_select;
    end

    if ~isempty(Ymax)
        A_blocks{end+1} = [Gamma_extend_select, zeros(nyNp,Nslack)];
        b_blocks{end+1} = Ymax - Y0_select;
    end

    if ~isempty(deltaUmax)
        A_blocks{end+1} = [eye(Ndu), zeros(Ndu,Nslack)];
        b_blocks{end+1} = deltaUmax;

        A_blocks{end+1} = [-eye(Ndu), zeros(Ndu,Nslack)];
        b_blocks{end+1} = deltaUmax;
    end

    if dsafe0 > 0
        A_blocks{end+1} = LeftHandDebris;
        b_blocks{end+1} = RightHandDebris;
    end

    A = vertcat(A_blocks{:});
    b = vertcat(b_blocks{:});

end

%% Linear-system helpers

function integral_gammahat = integral_gammahat(A, h)
    n = size(A,1);
    M = [A, eye(n);
         zeros(n), zeros(n)];
    EM = expm(M * h);
    integral_gammahat = EM(1:n, n+1:end);
end


%% Base-workspace and data-shape helpers

function value = getBaseWorkspaceVar(varName, defaultValue)
    if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
        value = evalin('base', varName);
    else
        if nargin < 2
            error('Variable "%s" not found in base workspace.', varName);
        end
        value = defaultValue;
    end
end

function M = resolveWeightMatrix(fullName, stepName, defaultStep, dim, Np)

    if evalin('base', sprintf('exist(''%s'', ''var'')', fullName))
        M = evalin('base', fullName);
        return;
    end

    if evalin('base', sprintf('exist(''%s'', ''var'')', stepName))
        step = evalin('base', stepName);
    else
        step = defaultStep;
    end

    step = step(:).';

    if numel(step) == dim
        M = diag(repmat(step, 1, Np));
    elseif numel(step) == dim*Np
        M = diag(step);
    else
        error('Invalid size for %s/%s.', fullName, stepName);
    end
end

function v = resolveBoundVector(fullName, stepName, dim, Np, defaultValue)

    if evalin('base', sprintf('exist(''%s'', ''var'')', fullName))
        v = evalin('base', fullName);
        v = v(:);
        return;
    end

    if evalin('base', sprintf('exist(''%s'', ''var'')', stepName))
        step = evalin('base', stepName);
        step = step(:);

        if isempty(step)
            v = [];
        elseif nargin >= 3 && ~isempty(dim) && numel(step) == dim
            v = repmat(step, Np, 1);
        else
            v = step;
        end
    else
        v = defaultValue;
    end
end

function idx = resolveStateConstraintIdx(Ymin, Ymax, nx, Np)

    if evalin('base', 'exist(''stateConstraintIdx'', ''var'')')
        idx = evalin('base', 'stateConstraintIdx');
        idx = idx(:);
        return;
    end

    if ~isempty(Ymin)
        ny = numel(Ymin)/Np;
    elseif ~isempty(Ymax)
        ny = numel(Ymax)/Np;
    else
        idx = [];
        return;
    end

    idx = (1:ny).';

    if any(idx > nx)
        error('stateConstraintIdx contains indices larger than nx.');
    end
end

function P = coerceCovarianceMatrix(rawCovariance, nx)

    if isempty(rawCovariance)
        P = zeros(nx);
        return;
    end

    if isvector(rawCovariance)
        values = rawCovariance(:);

        if numel(values) == nx
            P = diag(values);
        elseif numel(values) == 9
            P = zeros(nx);
            P(1:3,1:3) = reshape(values, 3, 3);
        elseif numel(values) == nx * nx
            P = reshape(values, nx, nx);
        else
            error('Unsupported covariance vector size: %d', numel(values));
        end
    else
        [nRows, nCols] = size(rawCovariance);

        if nRows == nx && nCols == nx
            P = rawCovariance;
        elseif nRows == 3 && nCols == 3
            P = zeros(nx);
            P(1:3,1:3) = rawCovariance;
        else
            error('Unsupported covariance matrix size: %dx%d', nRows, nCols);
        end
    end

    P = symmetrizeCovariance(P);
end

function P = resolveCovarianceMatrix(candidateNames, nx, defaultValue)

    P = defaultValue;

    for i = 1:numel(candidateNames)
        varName = candidateNames{i};

        if evalin('base', sprintf('exist(''%s'', ''var'')', varName))
            P = coerceCovarianceMatrix(evalin('base', varName), nx);
            return;
        end
    end
end

function radius = covarianceRadiusFromPosition(P_pos, metric)

    P_pos = symmetrizeCovariance(P_pos);

    switch lower(metric)
        case 'sqrt_lambda_max_pos'
            radius = sqrt(max(max(real(eig(P_pos))), 0));
        otherwise
            radius = sqrt(max(trace(P_pos), 0));
    end
end

function x_hist = coerceStateHistory(rawHistory, nx)

    if isempty(rawHistory)
        x_hist = [];
        return;
    end

    if isvector(rawHistory)
        values = rawHistory(:);
        if mod(numel(values), nx) ~= 0
            error('Unsupported state-history vector size: %d', numel(values));
        end
        x_hist = reshape(values, nx, []);
        return;
    end

    [nRows, nCols] = size(rawHistory);

    if nRows == nx
        x_hist = rawHistory;
    elseif nCols == nx
        x_hist = rawHistory.';
    else
        error('Unsupported state-history matrix size: %dx%d', nRows, nCols);
    end
end

function r_debris = getDebrisPositionAtStep(x_debris_hist, step, nx, rk_debris)

    if ~isempty(x_debris_hist)
        step = max(1, min(step, size(x_debris_hist, 2)));
        r_debris = x_debris_hist(1:3, step);
    else
        r_debris = rk_debris(:);
    end
end

function P = symmetrizeCovariance(P)
    P = 0.5 * (P + P.');
end

function S = skewSymmetric(v)
    S = [0,    -v(3),  v(2);
         v(3),  0,    -v(1);
        -v(2),  v(1),  0];
end

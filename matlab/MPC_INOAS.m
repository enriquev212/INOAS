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
    persistent diag_log
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
        diag_log = emptyDiagLog();
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


    %% Prediction matrices (cached)
    % Phi, Gamma, Phi_extend, E, H, Gamma_u and Gamma_extend depend only on the
    % pair (h, Np) and on the model, never on the state, yet all of them used to
    % be rebuilt from scratch on every call, with Phi^i recomputed by repeated
    % exponentiation thousands of times per solve. Build once per configuration.
    pred = getPredictionMatrices(A_c, B_c, h, Np, nx, m);
    Phi          = pred.Phi;
    Gamma        = pred.Gamma;
    Phi_extend   = pred.Phi_extend;
    E            = pred.E;
    H            = pred.H;
    Gamma_u      = pred.Gamma_u;
    Gamma_extend = pred.Gamma_extend;

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
    Q_eff = Q;   % puede relajarse durante la ventana de evasion
    LeftHandDebris = zeros(Np, m*Np + Np);
    RightHandDebris = zeros(Np,1);
    debrisScale = ones(Np,1);

    if dsafe0 > 0 && strcmpi(cfg.debrisMode, "bplane_tca")
        % ---------------- B-plane constraint at closest approach --------
        % One row instead of Np. The remaining rows are left trivially
        % inactive (0 <= 1) so that the block structure of A stays fixed.
        RightHandDebris(:) = 1;
        dsafe_profile = zeros(Np,1);

        dt_tca = cfg.tTca - (timeStep-1)*h;    % from the current grid time
        if ~isempty(cfg.tTca) && dt_tca > 0 && dt_tca <= Np*h
            iStar = min(max(floor(dt_tca/h), 0), Np-1);
            tau   = dt_tca - iStar*h;

            % State at the encounter: propagate the last grid point by the
            % remaining fraction of a step. The encounter almost never falls
            % on a grid node, and at km/s that fraction matters at metre level.
            Phi_tau = expm(A_c*tau);
            Gam_tau = integral_gammahat(A_c, tau) * B_c;

            if iStar >= 1
                rows_i = (iStar-1)*nx + (1:nx);
                Y0_i   = Y0(rows_i);
                G_i    = Gamma_extend(rows_i,:);
                Hrow   = H((iStar-1)*m + (1:m), :);
            else
                Y0_i = x_rel_estim;  G_i = zeros(nx, m*Np);  Hrow = zeros(m, m*Np);
            end

            Y0_tca = Phi_tau*Y0_i + Gam_tau*u;
            G_tca  = Phi_tau*G_i  + Gam_tau*Hrow;
            r0_tca = Y0_tca(1:3);
            Gr_tca = G_tca(1:3,:);

            % Control applied after the deadline does not count towards
            % satisfying the constraint, so the optimizer cannot defer.
            n_dl = Np;
            if ~isempty(cfg.tDeadline)
                n_dl = floor((cfg.tDeadline - (timeStep-1)*h)/h);
                n_dl = min(n_dl, Np);
                if n_dl < Np && n_dl > 0
                    Gr_tca(:, n_dl*m+1:end) = 0;
                end
            end
            % Pasada la fecha limite no queda autoridad util sobre el
            % encuentro: imponer la restriccion solo produce holgura que el
            % optimizador absorbe sin actuar. Se retira.
            imposeDebris = (n_dl > 0);

            % Encounter geometry, brought into the frame the controller works in.
            d_nom_t = T_abs_to_ref * cfg.dNomEci(:);
            u_rel_t = T_abs_to_ref * cfg.uRelEci(:);
            u_rel_t = u_rel_t / norm(u_rel_t);
            Proj    = eye(3) - (u_rel_t*u_rel_t.');    % onto the B-plane

            q  = Proj * (d_nom_t + r0_tca);            % predicted miss vector
            Mq = Proj * Gr_tca;                        % its sensitivity to dU

            % Navigation covariance propagated to the encounter, combined with
            % the object's. Until now only the spacecraft's was used, which is
            % why the radius could not be read as a collision probability.
            if ~isempty(cfg.PnavTca)
                P_nav_pos = T_abs_to_ref * cfg.PnavTca * T_abs_to_ref.';
            elseif cfg.vanLoanQ
                Q_c_eff = cfg.Q_c;
                if ~any(Q_c_eff(:)); Q_c_eff = Q_cov / cfg.QcReferenceDt; end
                Phi_dt = expm(A_c*dt_tca);
                P_tca  = Phi_dt*P_mpc*Phi_dt.' + vanLoanProcessNoise(A_c, Q_c_eff, dt_tca);
                P_nav_pos = symmetrizeCovariance(P_tca(1:3,1:3));
            else
                nSteps = max(1, round(dt_tca/h));
                P_tca = P_mpc;
                for kk = 1:nSteps
                    P_tca = Phi*P_tca*Phi.' + Q_cov;
                end
                P_nav_pos = symmetrizeCovariance(P_tca(1:3,1:3));
            end

            P_deb_ref = T_abs_to_ref * cfg.Pdebris * T_abs_to_ref.';
            P_comb    = P_nav_pos + P_deb_ref;

            % Margin along the miss direction, inside the B-plane. Same shape
            % as the previous d_safe = d0 + k*sigma, now with both objects.
            nq = norm(q);
            if nq > 1e-9
                u_b = q / nq;
            else
                u_b = Proj(:,1); u_b = u_b/max(norm(u_b),eps);
            end
            sigma_b = sqrt(max(u_b.' * P_comb * u_b, 0));
            dsafe_k = dsafe0 + cfg.kSigma * sigma_b;
            dsafe_profile(:) = dsafe_k;

            if imposeDebris
                % Relajar el seguimiento hasta el encuentro, inclusive. Los
                % pasos posteriores conservan el peso completo, de modo que
                % la maniobra de retorno sigue estando en el problema.
                if cfg.trackRelax ~= 1
                    nRelax = min(iStar+1, Np);
                    for iq = 1:nRelax
                        idq = (iq-1)*nx + (1:nx);
                        Q_eff(idq,idq) = Q(idq,idq) * cfg.trackRelax;
                    end
                end

                scaleDebris = max([nq^2, dsafe_k^2, 1]);
                debrisScale(1) = scaleDebris;
                LeftHandDebris(1,1:m*Np) = (-2*q.' * Mq) / scaleDebris;
                LeftHandDebris(1,m*Np+1) = -1 / scaleDebris;
                RightHandDebris(1)       = (nq^2 - dsafe_k^2) / scaleDebris;
            end
        end

        if logDsafe
            dsafe_log_time(end+1,1)  = t_sim;
            dsafe_log_first(end+1,1) = dsafe_profile(1);
            dsafe_log_max(end+1,1)   = max(dsafe_profile);
            assignin('base', 'mpc_dsafe_log_time', dsafe_log_time);
            assignin('base', 'mpc_dsafe_log_first', dsafe_log_first);
            assignin('base', 'mpc_dsafe_log_max', dsafe_log_max);
        end

    elseif dsafe0 > 0
        dsafe_profile = zeros(Np,1);
        debrisScale = ones(Np,1);   % escala de cada fila, para normalizar el slack

        % Process noise for ONE prediction step of length h. Adding a fixed
        % Q_cov once per step made the accumulated uncertainty proportional
        % to the NUMBER of steps instead of to the elapsed time, so two
        % configurations covering the same 500 s horizon disagreed by 12.5%
        % on the inflated radius. Discretizing a continuous PSD over h makes
        % the keep-out zone a property of the physics rather than of the mesh.
        if cfg.vanLoanQ
            Q_c_eff = cfg.Q_c;
            if ~any(Q_c_eff(:))
                % Legacy input: a discrete covariance calibrated at QcReferenceDt.
                Q_c_eff = Q_cov / cfg.QcReferenceDt;
            end
            Q_step = vanLoanProcessNoise(A_c, Q_c_eff, h);
        else
            Q_step = Q_cov;   % previous behaviour, dose per step
        end

        P_k = P_mpc;
        for i = 1:Np
            P_k = Phi * P_k * Phi.' + Q_step;
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
            debrisScale(i) = scaleDebris;

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

    % The slack column of the debris rows is -1/scaleDebris, so the slack
    % variable carries units of m^2 and takes values of 1e4 to 1e6 while
    % delta_U is around 1e-3. Left unscaled the QP spans some nine orders of
    % magnitude and the interior-point method stalls. Normalising each slack by
    % its own row factor is an exact change of variables: same optimum, far
    % better conditioned, and it turns those -1/scaleDebris entries into -1.
    s_scale = debrisScale(:);
    s_scale(~isfinite(s_scale) | s_scale <= 0) = 1;
    Dsl = diag(s_scale);

    w0 = Ddu \ delta_U0;

    slack0 = zeros(Nslack,1);
    z0 = [w0; slack0];

    lb_w = lb_deltaU ./ du_scale;
    ub_w = ub_deltaU ./ du_scale;

    lb = [lb_w; lb_slack ./ s_scale];
    ub = [ub_w; ub_slack ./ s_scale];

    A_scaled = A;
    A_scaled(:,1:Ndu) = A(:,1:Ndu) * Ddu;
    A_scaled(:,Ndu+1:end) = A(:,Ndu+1:end) * Dsl;

    %% Solve constrained MPC problem
    % This is a strictly convex QP: quadratic objective with a constant, known
    % Hessian, linear inequalities and simple bounds. It used to be handed to
    % fmincon with the 'sqp' algorithm, a general nonlinear solver. The measured
    % worst case was 80 s for a 400-variable instance and is unbounded in
    % general, which is not acceptable for a controller that must return within
    % h seconds.
    %
    %   J(z) = 0.5*z'*Hqp*z + fqp'*z + const,   z = [w; slack],  delta_U = Ddu*w
    %
    % The constant term is dropped: it does not move the argmin, but it does
    % shift fval with respect to the old objective value.
    U0 = repmat(u, Np, 1);
    Hdu = Gamma_extend.'*Q_eff*Gamma_extend + S + H.'*R*H;
    fdu = Gamma_extend.'*(Q_eff*Y0) + H.'*(R*U0);

    Hqp = blkdiag(Ddu.'*Hdu*Ddu, 2*slackWeight*(Dsl.'*Dsl));
    Hqp = 0.5*(Hqp + Hqp.');
    fqp = [Ddu.'*fdu; zeros(Nslack,1)];

    options = optimoptions('quadprog', ...
        'Display','none', ...
        'Algorithm', char(cfg.qpAlgorithm), ...
        'OptimalityTolerance',1e-8, ...
        'ConstraintTolerance',1e-8, ...
        'MaxIterations',2000);

    solveTimer = tic;
    [z_opt,fval,exitflag,output] = quadprog(Hqp,fqp,A_scaled,b,[],[],lb,ub,z0,options);
    solveTime = toc(solveTimer);

    if isempty(z_opt)
        % Hold the warm start rather than propagate an empty solution.
        warning('INOAS:qpFailed', ...
            'quadprog returned no solution at t = %.3f s (exitflag %d). Holding.', ...
            t_sim, exitflag);
        z_opt = z0;
    end
    
    w_opt = z_opt(1:Ndu);
    delta_U = Ddu*w_opt;
    z_opt(Ndu+1:end) = Dsl * z_opt(Ndu+1:end);   % slack de vuelta a m^2

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

    % The QP bounds each component in the LVLH/reference frame, but this clamp
    % bounds each component in ECI. Neither bounds the norm, so the commanded
    % magnitude can exceed u_max by up to sqrt(3) without any component
    % saturating. Keep the pre-clamp value so that the bite is measurable.
    u_abs_unclamped = u_abs;
    u_abs_max = cfg.Umax(1);
    if strcmpi(cfg.uLimitMode, "norm")
        % Bound the magnitude, preserving direction: the manoeuvre keeps
        % pointing where the optimizer wanted, only shorter.
        nrm = norm(u_abs);
        if nrm > u_abs_max
            u_abs = u_abs * (u_abs_max / nrm);
        end
    else
        u_abs = max(min(u_abs, u_abs_max), -u_abs_max);
    end

    %% Solver and actuator diagnostics
    if isempty(diag_log)
        diag_log = emptyDiagLog();
    end
    diag_log.t(end+1,1)              = t_sim;
    diag_log.solve_time(end+1,1)     = solveTime;
    diag_log.exitflag(end+1,1)       = exitflag;
    diag_log.iterations(end+1,1)     = output.iterations;
    if isfield(output, 'funcCount')
        diag_log.funcCount(end+1,1)  = output.funcCount;
    else
        diag_log.funcCount(end+1,1)  = NaN;   % quadprog does not report it
    end
    diag_log.violation(end+1,1)      = viol;
    diag_log.fval(end+1,1)           = fval;
    diag_log.u_norm(end+1,1)         = norm(u_abs);
    diag_log.u_norm_unclamped(end+1,1) = norm(u_abs_unclamped);
    diag_log.clamp_bite(end+1,1)     = max(max(abs(u_abs_unclamped)) - u_abs_max, 0);
    diag_log.u_ref_max(end+1,1)      = max(abs(u_ref));
    diag_log.du_max_active(end+1,1)  = max(abs(delta_u)) >= 0.999*cfg.deltaUmax(1);
    diag_log.slack_max(end+1,1)      = max(slack_opt_internal);
    assignin('base', 'mpc_diag_log', diag_log);

    u_abs_current = u_abs;
    u = u_abs;

    if ~hasExternalTime
        t_internal = t_sim + sampleTime;
    else
        t_internal = t_sim;
    end

end



%% Diagnostics helpers

function log = emptyDiagLog()
%EMPTYDIAGLOG Empty solver/actuator diagnostics record.
    z = zeros(0,1);
    log = struct( ...
        't', z, 'solve_time', z, 'exitflag', z, 'iterations', z, ...
        'funcCount', z, 'violation', z, 'fval', z, 'u_norm', z, ...
        'u_norm_unclamped', z, 'clamp_bite', z, 'u_ref_max', z, ...
        'du_max_active', z, 'slack_max', z);
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
    cfg.Q_c = resolveCovarianceMatrix({'Q_c_mpc'}, n, zeros(n));
    cfg.QcReferenceDt = getBaseWorkspaceVar('Q_cov_reference_dt', 1);
    cfg.vanLoanQ = getBaseWorkspaceVar('mpcVanLoanQ', true);
    % 'active-set' aprovecha el warm start y converge en las instancias duras
    % (el cruce del debris, donde la restriccion se activa y el problema se
    % vuelve degenerado); 'interior-point-convex' sale ahi con exitflag -8.
    cfg.qpAlgorithm = string(getBaseWorkspaceVar('mpcQpAlgorithm', "active-set"));

    % Debris-avoidance constraint mode.
    %   "sphere_grid" : ||r_rel|| >= d_safe at every prediction step. Works
    %                   only while the object crosses slowly enough for the
    %                   grid to resolve the encounter.
    %   "bplane_tca"  : a single constraint on the miss distance projected
    %                   onto the B-plane at the time of closest approach.
    %                   Independent of the grid, and the natural place for
    %                   the combined covariance of both objects.
    cfg.debrisMode = string(getBaseWorkspaceVar('mpcDebrisMode', "sphere_grid"));
    cfg.tTca       = getBaseWorkspaceVar('conj_t_tca', []);
    cfg.dNomEci    = getBaseWorkspaceVar('conj_d_nom_eci', []);
    cfg.uRelEci    = getBaseWorkspaceVar('conj_u_rel_eci', []);
    cfg.Pdebris    = getBaseWorkspaceVar('conj_P_debris', zeros(3));
    cfg.kSigma     = getBaseWorkspaceVar('conj_k_sigma', 3);
    % Predicted navigation covariance AT the encounter, in ECI, supplied by
    % the navigation layer. Extrapolating the filter covariance over the full
    % lead time is not an option: Q_matrix is a filter tuning parameter, not a
    % physical power spectral density. Its velocity term of 1e-2 (m/s)^2/s
    % grows the uncertainty to kilometres over an orbit, which would drive the
    % keep-out radius to tens of km and saturate the actuator. The navigation
    % layer is the one that knows the duty-cycle plan and can predict it.
    cfg.PnavTca    = getBaseWorkspaceVar('conj_P_nav_tca', []);
    % Manoeuvre deadline. A receding-horizon controller with a terminal
    % avoidance constraint procrastinates: at every step, acting later is
    % still feasible and the control cost rewards waiting, so it defers until
    % the constraint is about to bite. That is the worst possible policy here,
    % because the lever arm of an along-track burn grows with the time left to
    % the encounter: the displacement it buys is 3*dv*t. Deferring the burn by
    % an orbit multiplies its cost. Requiring the manoeuvre to be complete by a
    % deadline removes the option to wait.
    cfg.tDeadline  = getBaseWorkspaceVar('conj_t_deadline', []);
    % Relajacion del seguimiento durante la evasion. El coste de seguimiento
    % pesa unas mil veces mas que el de control, asi que al controlador le
    % sale caro ESTAR fuera de la nominal y su optimo es desviarse lo mas
    % tarde posible. Eso es exactamente lo contrario de lo que debe hacer:
    % un impulso tangencial compra 3*dv*t de desplazamiento, de modo que
    % esperar multiplica el coste. Una vez comprometida la maniobra, la
    % trayectoria nominal ES la desplazada, y seguir la antigua no debe
    % costar nada. Relajar Q en la ventana convierte el problema en el de
    % delta-v minimo, que es el que se quiere resolver.
    cfg.trackRelax = getBaseWorkspaceVar('mpcTrackRelax', 1);
    cfg.uLimitMode = string(getBaseWorkspaceVar('uLimitMode', "per_axis"));
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

function pred = getPredictionMatrices(A_c, B_c, h, Np, nx, m)
%GETPREDICTIONMATRICES Prediction matrices for the pair (h, Np), cached.
%   Rebuilt only when the configuration actually changes. Powers of Phi are
%   formed incrementally and Gamma_u exploits its block-Toeplitz structure:
%   block (i,j) depends only on i-j, so each distinct block is formed once.
    persistent cacheKey cached

    key = [nx, m, Np, h, norm(A_c(:), 1), norm(B_c(:), 1)];

    if ~isempty(cacheKey) && isequal(size(cacheKey), size(key)) && ...
            all(abs(cacheKey - key) <= 1e-12 * max(1, abs(key)))
        pred = cached;
        return;
    end

    Phi = expm(A_c*h);
    Gamma = integral_gammahat(A_c, h) * B_c;

    Phi_pow = cell(Np, 1);
    Phi_pow{1} = Phi;
    for i = 2:Np
        Phi_pow{i} = Phi_pow{i-1} * Phi;
    end

    Phi_extend = zeros(Np*nx, nx);
    for i = 1:Np
        Phi_extend((i-1)*nx + (1:nx), :) = Phi_pow{i};
    end

    E = repmat(eye(m), Np, 1);
    H = kron(tril(ones(Np)), eye(m));

    blocks = cell(Np, 1);
    blocks{1} = Gamma;
    for d = 2:Np
        blocks{d} = Phi_pow{d-1} * Gamma;
    end

    Gamma_u = zeros(Np*nx, Np*m);
    for i = 1:Np
        rows = (i-1)*nx + (1:nx);
        for j = 1:i
            Gamma_u(rows, (j-1)*m + (1:m)) = blocks{i-j+1};
        end
    end

    cached = struct('Phi', Phi, 'Gamma', Gamma, 'Phi_extend', Phi_extend, ...
        'E', E, 'H', H, 'Gamma_u', Gamma_u, 'Gamma_extend', Gamma_u * H);
    cacheKey = key;
    pred = cached;
end

function Q_d = vanLoanProcessNoise(A, Q_c, h)
%VANLOANPROCESSNOISE Discrete process noise over a step of length h.
%   Van Loan (1978): for xdot = A x + w with E[w w'] = Q_c delta(t), the
%   equivalent discrete covariance over h is obtained from a single matrix
%   exponential. Unlike a fixed per-step dose, this scales with the step
%   length, so the propagated uncertainty depends on elapsed time only.
    n = size(A,1);
    M = [-A, Q_c; zeros(n), A.'] * h;
    F = expm(M);
    Phi_T = F(n+1:2*n, n+1:2*n);
    Q_d = Phi_T.' * F(1:n, n+1:2*n);
    Q_d = 0.5 * (Q_d + Q_d.');
end

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

function P = plan_cam(x_ref_hist, t_ref, SC, opts)
%PLAN_CAM Plan a collision-avoidance manoeuvre and retarget the nominal.
%
%   A tracking controller cannot be asked to perform a collision-avoidance
%   manoeuvre against its own reference. The tracking term costs roughly a
%   thousand times more than the control term, so leaving the nominal is
%   expensive and the optimizer minimises the time spent away from it: it defers
%   the burn to the last moment, which is the worst possible policy, because an
%   along-track impulse buys 3*dv*t of displacement and waiting throws that lever
%   arm away. Removing the tracking term instead leaves the state unbounded.
%
%   The way operational systems do it, and the way this function does it, is to
%   separate the two jobs. Guidance decides the manoeuvre and produces a NEW
%   nominal trajectory that already contains it. The controller then does what it
%   is good at: tracking. Holding the manoeuvre costs nothing, because the
%   manoeuvre is the reference.
%
%   The burn magnitude is solved on the real propagated dynamics, not on the
%   linear model, by secant iteration on the achieved miss distance.
%
%   opts:
%     d_target   required miss distance at closest approach [m]
%     t_burn     time of the manoeuvre [s]
%     dv_seed    initial guess for the impulse [m/s], default 20e-3
%
%   Returns the impulse, the retargeted reference stacked like r_p_full, and the
%   achieved miss distance.

    arguments
        x_ref_hist double
        t_ref      double
        SC         struct
        opts.d_target (1,1) double
        opts.t_burn   (1,1) double
        opts.dv_seed  (1,1) double = 20e-3
    end

    if size(x_ref_hist,1) ~= 6 && size(x_ref_hist,2) == 6
        x_ref_hist = x_ref_hist.';
    end
    t_ref = t_ref(:);
    odeo = odeset('RelTol',1e-11,'AbsTol',1e-9);

    % --- state on the nominal at the burn time ------------------------------
    [~, iB] = min(abs(t_ref - opts.t_burn));
    t_burn  = t_ref(iB);
    x_b     = x_ref_hist(:, iB);

    % Object state at the same instant, so both are referred to one epoch
    x_d_b = SC.x_debris_hist(:, iB);

    % --- solve for the impulse that reaches the required miss ---------------
    dv = opts.dv_seed;
    [m_a, ~] = missWithBurn(x_b, x_d_b, t_burn, dv, SC.t_tca, odeo);
    m0 = missWithBurn(x_b, x_d_b, t_burn, 0, SC.t_tca, odeo);
    slope = (m_a - m0) / dv;
    if slope <= 0
        error('plan_cam:noAuthority', ...
            'A tangential burn does not open the miss distance in this geometry.');
    end

    dv_a = dv;  ma = m_a;
    dv_b = max(1e-9, (opts.d_target - m0)/slope);
    [mb, ~] = missWithBurn(x_b, x_d_b, t_burn, dv_b, SC.t_tca, odeo);
    for it = 1:15
        if abs(mb - opts.d_target) < 0.5; break; end
        den = mb - ma;
        if abs(den) < 1e-9; break; end
        dv_new = max(0, dv_b + (opts.d_target - mb)*(dv_b - dv_a)/den);
        dv_a = dv_b;  ma = mb;
        dv_b = dv_new;
        [mb, ~] = missWithBurn(x_b, x_d_b, t_burn, dv_b, SC.t_tca, odeo);
    end

    % --- retargeted nominal: propagate the post-burn state forward ----------
    v_hat = x_b(4:6)/norm(x_b(4:6));
    x_post = x_b;  x_post(4:6) = x_post(4:6) + dv_b*v_hat;

    x_new = x_ref_hist;
    xk = x_post;
    for k = iB+1:numel(t_ref)
        [~, XX] = ode113(@(t,x) inoas_dyn(t,x,zeros(3,1)), ...
                         [t_ref(k-1) t_ref(k)], xk, odeo);
        xk = XX(end,:).';
        x_new(:,k) = xk;
    end
    x_new(:, iB) = x_post;

    P = struct();
    P.dv          = dv_b;
    P.dv_vec      = dv_b * v_hat;
    P.t_burn      = t_burn;
    P.i_burn      = iB;
    P.miss_before = m0;
    P.miss_after  = mb;
    P.x_ref_new   = x_new;
    P.r_p_full    = reshape(x_new, [], 1);
    P.burn_time_s = [];    % filled by the caller if the thruster is known
end

function [mis, T] = missWithBurn(x_b, x_d_b, t_burn, dv, t_tca, odeo) %#ok<INUSD>
    xb = x_b;
    if dv ~= 0
        v_hat = xb(4:6)/norm(xb(4:6));
        xb(4:6) = xb(4:6) + dv*v_hat;
    end
    T = cam_find_tca(xb, x_d_b, t_burn, t_tca, 120);
    mis = T.miss;
end

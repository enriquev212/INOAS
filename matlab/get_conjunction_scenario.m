function S = get_conjunction_scenario(h, x_ref_hist, t_ref, t_tca_nom, opts)
%GET_CONJUNCTION_SCENARIO Realistic plane-crossing conjunction on the same orbit.
%
%   get_debris_trajectory builds the object as an LVLH offset with a chosen
%   relative velocity. Setting that velocity to 10 m/s, as the baseline scenario
%   does, implies a plane separation of 0.08 deg: an essentially co-orbital
%   object, not a conjunction. Real LEO conjunctions run at several km/s.
%
%   Here the object is placed on an orbit of its own. Its velocity at the time
%   of closest approach is the spacecraft velocity rotated about the local
%   radial direction by the crossing angle. That rotation preserves |v| and the
%   angle between r and v, so the object keeps the same semi-major axis and
%   eccentricity and only its plane differs. The relative velocity then follows
%   from the geometry:
%
%       |v_rel| = 2 |v| sin(dInc/2)
%
%   The function also returns everything the guidance layer needs to impose the
%   constraint at the time of closest approach rather than on the prediction
%   grid, which at km/s is the only workable option: the pass through a 150 m
%   sphere lasts tens of milliseconds, while the grid steps are seconds.
%
%   opts fields (all optional):
%     dInc_deg     plane-crossing angle [deg], default 88.15 (~10 km/s)
%     miss_m       unperturbed closest-approach distance [m], default 120
%     bplane_deg   orientation of the miss vector within the B-plane [deg]
%     P_debris     object position covariance in ECI [3x3], default catalogue-like
%     R_hb         combined hard-body radius [m], default 5

    arguments
        h           (1,1) double
        x_ref_hist  double
        t_ref       double
        t_tca_nom   (1,1) double
        opts.dInc_deg   (1,1) double = 88.15
        opts.miss_m     (1,1) double = 120
        opts.bplane_deg (1,1) double = 25
        opts.P_debris   (3,3) double = diag([150 60 60].^2)
        opts.R_hb       (1,1) double = 5
    end

    mu = 3.986004418e14;
    if size(x_ref_hist,1) ~= 6 && size(x_ref_hist,2) == 6
        x_ref_hist = x_ref_hist.';
    end
    t_ref = t_ref(:);

    % --- spacecraft state at the nominal time of closest approach -----------
    [~, iTca] = min(abs(t_ref - t_tca_nom));
    x_c = x_ref_hist(:, iTca);
    t_tca = t_ref(iTca);
    r_c = x_c(1:3);  v_c = x_c(4:6);

    % --- object orbit: rotate v_c about the radial direction ----------------
    r_hat = r_c / norm(r_c);
    th = deg2rad(opts.dInc_deg);
    K = [      0     -r_hat(3)  r_hat(2);
          r_hat(3)       0     -r_hat(1);
         -r_hat(2)  r_hat(1)       0    ];
    Rrot = eye(3) + sin(th)*K + (1-cos(th))*(K*K);      % Rodrigues
    v_d = Rrot * v_c;

    v_rel = v_d - v_c;
    u_hat = v_rel / norm(v_rel);

    % --- miss vector, inside the B-plane ------------------------------------
    tmp = cross(v_d, v_c);
    if norm(tmp) < 1e-9; tmp = cross(u_hat, r_hat); end
    xi_hat = tmp - (tmp.'*u_hat)*u_hat;
    xi_hat = xi_hat / norm(xi_hat);
    zeta_hat = cross(u_hat, xi_hat);

    phi = deg2rad(opts.bplane_deg);
    b_vec = opts.miss_m * (cos(phi)*xi_hat + sin(phi)*zeta_hat);
    r_d = r_c + b_vec;

    % --- propagate the object over the whole reference timeline -------------
    % Backwards from the encounter to t_ref(1), then forwards to the end, so
    % that the legacy grid-sampled path and the plotting utilities keep working.
    x_d_tca = [r_d; v_d];
    N = numel(t_ref);
    x_debris_hist = zeros(6, N);
    x_debris_hist(:, iTca) = x_d_tca;

    for k = iTca+1:N
        x_debris_hist(:,k) = rk4Step(x_debris_hist(:,k-1), t_ref(k)-t_ref(k-1));
    end
    for k = iTca-1:-1:1
        x_debris_hist(:,k) = rk4Step(x_debris_hist(:,k+1), t_ref(k)-t_ref(k+1));
    end

    % --- data for the closest-approach constraint ---------------------------
    S = struct();
    S.x_debris_hist = x_debris_hist;
    S.r_debris_full = reshape(x_debris_hist, [], 1);
    S.t_debris_ref  = t_ref;
    S.encounter_idx = iTca;
    S.rk_debris_encounter = r_d;

    S.t_tca      = t_tca;
    S.d_nom_eci  = r_c - r_d;              % spacecraft minus object, ECI
    S.u_rel_eci  = u_hat;                  % encounter axis, ECI
    S.v_rel_mag  = norm(v_rel);
    S.xi_eci     = xi_hat;
    S.zeta_eci   = zeta_hat;
    S.P_debris   = opts.P_debris;
    S.R_hb       = opts.R_hb;
    S.dInc_deg   = opts.dInc_deg;
    S.miss_nom   = norm(b_vec);
    S.a_chaser   = 1/(2/norm(r_c) - norm(v_c)^2/mu);
    S.a_debris   = 1/(2/norm(r_d) - norm(v_d)^2/mu);
    % How long the object spends inside a sphere of the nominal radius. This is
    % the number that rules out sampling the constraint on the prediction grid.
    S.pass_time  = 2*opts.miss_m / S.v_rel_mag;
end

function x_next = rk4Step(x, dt)
%RK4STEP Advance by dt, substepping so the internal step stays around 1 s.
%   The guidance grid can be 60 s wide, which is far too coarse for RK4 to keep
%   the object's position accurate to metres over an orbit or more. The miss
%   distance is a metre-level quantity, so the propagation cannot inherit the
%   grid resolution.
    nsub = max(1, ceil(abs(dt)/1.0));
    ds = dt / nsub;
    x_next = x;
    for i = 1:nsub
        k1 = deriv(x_next);
        k2 = deriv(x_next + 0.5*ds*k1);
        k3 = deriv(x_next + 0.5*ds*k2);
        k4 = deriv(x_next + ds*k3);
        x_next = x_next + (ds/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

function xd = deriv(x)
    mu = 3.986004418e14; Re = 6378137.0; J2 = 1.08262668e-3;
    r = x(1:3); rn = norm(r);
    a2b = -(mu/rn^3)*r;
    zr = r(3)/rn; kk = 1.5*J2*mu*Re^2/rn^4;
    aJ2 = -kk*[ (1-5*zr^2)*r(1)/rn; (1-5*zr^2)*r(2)/rn; (3-5*zr^2)*r(3)/rn ];
    xd = [x(4:6); a2b + aJ2];
end

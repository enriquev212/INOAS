function get_debris_trajectory(Ts, x_ref_hist, t_ref, t_encounter, rel_pos_lvlh, rel_vel_lvlh, filename)

    if nargin < 7 || isempty(filename)
        filename = "debrisTrajectory.mat";
    end

    rel_pos_lvlh = rel_pos_lvlh(:);
    rel_vel_lvlh = rel_vel_lvlh(:);

    if numel(rel_pos_lvlh) ~= 3 || numel(rel_vel_lvlh) ~= 3
        error('rel_pos_lvlh and rel_vel_lvlh must be 3x1 vectors.');
    end

    if size(x_ref_hist,1) ~= 6 && size(x_ref_hist,2) == 6
        x_ref_hist = x_ref_hist.';
    end

    if size(x_ref_hist,1) ~= 6
        error('x_ref_hist must be a 6xN history of the reference trajectory.');
    end

    t_ref = t_ref(:);
    if numel(t_ref) ~= size(x_ref_hist,2)
        error('t_ref length must match the number of reference states.');
    end

    [~, encounter_idx] = min(abs(t_ref - t_encounter));

    x_ref_enc = x_ref_hist(:, encounter_idx);
    r_ref_enc = x_ref_enc(1:3);
    v_ref_enc = x_ref_enc(4:6);

    [~, T_ref_to_abs] = referenceFrameTransform(r_ref_enc, v_ref_enc);

    h_ref_vec = cross(r_ref_enc, v_ref_enc);
    n_ref = norm(h_ref_vec) / norm(r_ref_enc)^2;
    omega_lvlh = [0; 0; n_ref];

    r_debris_enc = r_ref_enc + T_ref_to_abs * rel_pos_lvlh;
    v_debris_enc = v_ref_enc + T_ref_to_abs * (rel_vel_lvlh + cross(omega_lvlh, rel_pos_lvlh));

    x_debris_enc = [r_debris_enc; v_debris_enc];

    x_debris_hist = zeros(6, numel(t_ref));
    x_debris_hist(:, encounter_idx) = x_debris_enc;

    for k = encounter_idx+1:numel(t_ref)
        dt = t_ref(k) - t_ref(k-1);
        x_debris_hist(:, k) = rk4Step(x_debris_hist(:, k-1), dt);
    end

    for k = encounter_idx-1:-1:1
        dt = t_ref(k) - t_ref(k+1);
        x_debris_hist(:, k) = rk4Step(x_debris_hist(:, k+1), dt);
    end

    r_debris_full = reshape(x_debris_hist, [], 1);
    rk_debris_encounter = r_debris_enc;
    t_debris_ref = t_ref;

    save(filename, 'x_debris_hist', 'r_debris_full', 't_debris_ref', ...
        'rk_debris_encounter', 'encounter_idx', 't_encounter', ...
        'rel_pos_lvlh', 'rel_vel_lvlh');
end

function x_next = rk4Step(x_current, dt)
    k1 = debrisStateDerivative(x_current);
    k2 = debrisStateDerivative(x_current + 0.5 * dt * k1);
    k3 = debrisStateDerivative(x_current + 0.5 * dt * k2);
    k4 = debrisStateDerivative(x_current + dt * k3);

    x_next = x_current + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

function x_dot = debrisStateDerivative(x)
    mu_earth = 3.986004418e14;
    R_earth = 6378137.0;

    J = [1.08262668e-3, 0, 0];
    n_vals = [2, 4, 6];

    r_vec = x(1:3);
    v_vec = x(4:6);

    z = r_vec(3);
    r = norm(r_vec);
    u_z = z / r;
    z_hat = [0; 0; 1];

    a_central = -(mu_earth / r^3) * r_vec;
    a_zonales = zeros(3,1);

    for i = 1:numel(n_vals)
        n = n_vals(i);
        Jn = J(i);

        if Jn == 0
            continue;
        end

        if n == 2
            Pn  = 0.5 * (3*u_z^2 - 1);
            dPn = 3*u_z;
        elseif n == 4
            Pn  = (1/8) * (35*u_z^4 - 30*u_z^2 + 3);
            dPn = 0.5 * (140*u_z^3 - 60*u_z);
        elseif n == 6
            Pn  = (1/16) * (231*u_z^6 - 315*u_z^4 + 105*u_z^2 - 5);
            dPn = (1/16) * (1386*u_z^5 - 1260*u_z^3 + 210*u_z);
        else
            Pn = 0;
            dPn = 0;
        end

        scale = (mu_earth / r^2) * (R_earth / r)^n * Jn;
        bracket = (n + 1) * Pn * (r_vec / r) - dPn * ((z / r^2) * r_vec - z_hat);
        a_zonales = a_zonales + scale * bracket;
    end

    x_dot = [v_vec; a_central + a_zonales];
end

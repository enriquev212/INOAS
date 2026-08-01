function x_next = myStateTransitionFcn(x_current, u)
    % State-transition function for the Unscented Kalman Filter.
    %
    % The UKF requires a fixed-step, code-generation-compatible propagation
    % model. This function therefore uses a fourth-order Runge-Kutta integrator
    % instead of variable-step ODE solvers.
    %
    % The dynamics model combines central Keplerian gravity with optional zonal
    % harmonic perturbations. The current configuration keeps J2 enabled and
    % leaves J4/J6 at zero.

    % Force the control input to be a 3x1 column vector.
    u = u(:);

    dt = 1; % [s] Must match the Simulink estimator sample time.

    k1 = get_state_derivative(x_current, u);
    k2 = get_state_derivative(x_current + 0.5 * dt * k1, u);
    k3 = get_state_derivative(x_current + 0.5 * dt * k2, u);
    k4 = get_state_derivative(x_current + dt * k3, u);

    x_next = x_current + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

function x_dot = get_state_derivative(x, u)
    mu_earth = 3.986004418e14; % [m^3/s^2]
    R_earth = 6378137.0;       % [m]

    zonal_coefficients = [1.08262668e-3, 0, 0]; % [J2, J4, J6]
    harmonic_orders = [2, 4, 6];

    r_vec = x(1:3);
    r_vec = r_vec(:);
    v_vec = x(4:6);
    v_vec = v_vec(:);

    z = r_vec(3);
    r = norm(r_vec);

    u_z = z / r;
    z_hat = [0; 0; 1];

    central_accel = -(mu_earth / r^3) * r_vec;
    zonal_accel = zeros(3, 1);

    for i = 1:length(harmonic_orders)
        n = harmonic_orders(i);
        Jn = zonal_coefficients(i);

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
        direction = (n + 1) * Pn * (r_vec / r) - ...
            dPn * ((z / r^2) * r_vec - z_hat);

        zonal_accel = zonal_accel + scale * direction;
    end

    total_accel = central_accel + zonal_accel + u;
    x_dot = [v_vec; total_accel];
end

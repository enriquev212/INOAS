function profile = load_gnss_sensor_profile(filename, startDateJulian, varargin)
%LOAD_GNSS_SENSOR_PROFILE Build a GNSS sensor profile from the .dat file.
%
% The .dat trajectory is not used as the spacecraft state measurement here.
% It is used to estimate sampling, time-varying measurement covariance, DOP,
% satellite count, and a legacy replay trajectory for comparison.

    if nargin < 1 || strlength(string(filename)) == 0
        filename = "cov_perturb_POS_s6a_Y24D011_fixed.dat";
    end

    filename = inoas_data_file(filename);

    if nargin < 2 || isempty(startDateJulian)
        startDateJulian = juliandate(datetime(2024, 1, 11));
    end

    p = inputParser;
    p.addParameter("CovarianceWindow", 31);
    p.addParameter("SigmaFloorPosition", 0.05);
    p.addParameter("SigmaFloorVelocity", 0.005);
    p.addParameter("UseDopFloor", true);
    p.addParameter("VelocityFromPositionScale", sqrt(2));
    p.parse(varargin{:});
    opts = p.Results;

    if ~isfile(filename)
        error("GNSS profile file not found: %s", filename);
    end

    data = readmatrix(filename, "CommentStyle", "#");

    if isempty(data)
        error("GNSS profile file is empty: %s", filename);
    end

    if size(data, 2) < 17
        error("GNSS profile file must have at least 17 columns.");
    end

    data = data(data(:, 7) == 1, :);

    if isempty(data)
        error("No valid GNSS samples remain after filtering SOL == 1.");
    end

    t_sod   = data(:, 1);
    lon_deg = data(:, 2);
    lat_deg = data(:, 3);
    alt     = data(:, 4);
    sol     = data(:, 7);
    nsvvis  = data(:, 8);
    nsv     = data(:, 9);
    hpe     = data(:, 10);
    vpe     = data(:, 11);
    epe     = data(:, 12);
    npe     = data(:, 13);
    upe     = data(:, 14);
    hdop    = data(:, 15);
    vdop    = data(:, 16);
    pdop    = data(:, 17);

    t = t_sod - t_sod(1);
    dt = diff(t);
    sample_time = median(dt);

    if any(abs(dt - sample_time) > 1e-6)
        warning("GNSS profile has nonuniform sample time. Using median %.6f s.", sample_time);
    end

    [pos_ecef, vel_ecef] = lla_to_ecef_and_velocity(lat_deg, lon_deg, alt, t);
    [pos_eci, vel_eci] = ecef_history_to_eci(pos_ecef, vel_ecef, t);

    win = max(3, round(opts.CovarianceWindow));
    if mod(win, 2) == 0
        win = win + 1;
    end

    sigma_e_emp = local_moving_rms(epe, win);
    sigma_n_emp = local_moving_rms(npe, win);
    sigma_u_emp = local_moving_rms(upe, win);

    sigma_h_dop = zeros(size(hpe));
    sigma_v_dop = zeros(size(vpe));

    if opts.UseDopFloor
        uere_h = robust_median(hpe ./ max(hdop, eps));
        uere_v = robust_median(vpe ./ max(vdop, eps));

        if ~isfinite(uere_h) || uere_h <= 0
            uere_h = max(robust_rms(hpe), opts.SigmaFloorPosition);
        end

        if ~isfinite(uere_v) || uere_v <= 0
            uere_v = max(robust_rms(vpe), opts.SigmaFloorPosition);
        end

        sigma_h_dop = abs(hdop) .* uere_h;
        sigma_v_dop = abs(vdop) .* uere_v;
    end

    sigma_e = max(sigma_e_emp, sigma_h_dop ./ sqrt(2));
    sigma_n = max(sigma_n_emp, sigma_h_dop ./ sqrt(2));
    sigma_u = max(sigma_u_emp, sigma_v_dop);

    sigma_e = max(sigma_e, opts.SigmaFloorPosition);
    sigma_n = max(sigma_n, opts.SigmaFloorPosition);
    sigma_u = max(sigma_u, opts.SigmaFloorPosition);

    omega_E = 7.2921159e-5;
    n_samples = numel(t);

    R_pos_ecef = zeros(3, 3, n_samples);
    R_pos_eci = zeros(3, 3, n_samples);
    R_vel_eci = zeros(3, 3, n_samples);
    R_state_eci = zeros(6, 6, n_samples);
    R_state_eci_flat = zeros(n_samples, 36);
    R_pos_eci_flat = zeros(n_samples, 9);

    phi = deg2rad(lat_deg);
    lambda = deg2rad(lon_deg);

    for k = 1:n_samples
        C_neu_to_ecef = [ ...
            -sin(phi(k))*cos(lambda(k)), -sin(lambda(k)),  cos(phi(k))*cos(lambda(k)); ...
            -sin(phi(k))*sin(lambda(k)),  cos(lambda(k)),  cos(phi(k))*sin(lambda(k)); ...
             cos(phi(k)),                 0,               sin(phi(k))];

        R_neu = diag([sigma_n(k)^2, sigma_e(k)^2, sigma_u(k)^2]);
        R_pos_ecef(:, :, k) = C_neu_to_ecef * R_neu * C_neu_to_ecef.';

        theta = omega_E * t(k);
        C_ecef_to_eci = [ cos(theta), -sin(theta), 0; ...
                          sin(theta),  cos(theta), 0; ...
                          0,           0,          1];

        R_pos_eci(:, :, k) = C_ecef_to_eci * R_pos_ecef(:, :, k) * C_ecef_to_eci.';

        sigma_vn = max(opts.VelocityFromPositionScale * sigma_n(k) / sample_time, opts.SigmaFloorVelocity);
        sigma_ve = max(opts.VelocityFromPositionScale * sigma_e(k) / sample_time, opts.SigmaFloorVelocity);
        sigma_vu = max(opts.VelocityFromPositionScale * sigma_u(k) / sample_time, opts.SigmaFloorVelocity);

        R_vel_neu = diag([sigma_vn^2, sigma_ve^2, sigma_vu^2]);
        R_vel_ecef = C_neu_to_ecef * R_vel_neu * C_neu_to_ecef.';
        R_vel_eci(:, :, k) = C_ecef_to_eci * R_vel_ecef * C_ecef_to_eci.';

        R_state = zeros(6, 6);
        R_state(1:3, 1:3) = symmetrize_psd(R_pos_eci(:, :, k));
        R_state(4:6, 4:6) = symmetrize_psd(R_vel_eci(:, :, k));

        R_state_eci(:, :, k) = R_state;
        R_state_eci_flat(k, :) = reshape(R_state, 1, 36);
        R_pos_eci_flat(k, :) = reshape(R_state(1:3, 1:3), 1, 9);
    end

    ts_pos_eci = timeseries(pos_eci.', t);
    ts_pos_eci.Name = "GNSS_LegacyReplay_Position_ECI";
    ts_pos_eci.DataInfo.Units = "m";
    ts_pos_eci = setinterpmethod(ts_pos_eci, "zoh");

    ts_vel_eci = timeseries(vel_eci.', t);
    ts_vel_eci.Name = "GNSS_LegacyReplay_Velocity_ECI";
    ts_vel_eci.DataInfo.Units = "m/s";
    ts_vel_eci = setinterpmethod(ts_vel_eci, "zoh");

    ts_R_state_eci_flat = timeseries(R_state_eci_flat, t);
    ts_R_state_eci_flat.Name = "GNSS_MeasurementCovariance_State_ECI_flat36";
    ts_R_state_eci_flat.DataInfo.Units = "m2_and_m2s2";
    ts_R_state_eci_flat = setinterpmethod(ts_R_state_eci_flat, "zoh");

    ts_R_pos_eci_flat = timeseries(R_pos_eci_flat, t);
    ts_R_pos_eci_flat.Name = "GNSS_MeasurementCovariance_Position_ECI_flat9";
    ts_R_pos_eci_flat.DataInfo.Units = "m2";
    ts_R_pos_eci_flat = setinterpmethod(ts_R_pos_eci_flat, "zoh");

    start_datetime = datetime(startDateJulian, "ConvertFrom", "juliandate");

    profile = struct();
    profile.source = "GNSS sensor profile from dataset";
    profile.filename = string(filename);
    profile.frame = "ECI";
    profile.start_date_julian = startDateJulian;
    profile.start_datetime = start_datetime;
    profile.t = t;
    profile.t_sod = t_sod;
    profile.sample_time = sample_time;
    profile.valid_solution = sol;
    profile.nsv_visible = nsvvis;
    profile.nsv = nsv;
    profile.hdop = hdop;
    profile.vdop = vdop;
    profile.pdop = pdop;
    profile.errors_neu = [npe, epe, upe];
    profile.hpe = hpe;
    profile.vpe = vpe;
    profile.sigma_neu = [sigma_n, sigma_e, sigma_u];
    profile.pos_ecef = pos_ecef;
    profile.vel_ecef = vel_ecef;
    profile.pos_eci = pos_eci;
    profile.vel_eci = vel_eci;
    profile.initial_state_eci = [pos_eci(:, 1); vel_eci(:, 1)];
    profile.R_pos_ecef = R_pos_ecef;
    profile.R_pos_eci = R_pos_eci;
    profile.R_vel_eci = R_vel_eci;
    profile.R_state_eci = R_state_eci;
    profile.R_state_eci_flat = R_state_eci_flat;
    profile.R_pos_eci_flat = R_pos_eci_flat;
    profile.ts_pos_eci = ts_pos_eci;
    profile.ts_vel_eci = ts_vel_eci;
    profile.ts_R_state_eci_flat = ts_R_state_eci_flat;
    profile.ts_R_pos_eci_flat = ts_R_pos_eci_flat;
    profile.meta = struct( ...
        "source", profile.source, ...
        "filename", profile.filename, ...
        "sample_time", profile.sample_time, ...
        "covariance_window", win, ...
        "sigma_floor_position", opts.SigmaFloorPosition, ...
        "sigma_floor_velocity", opts.SigmaFloorVelocity, ...
        "n_samples", n_samples, ...
        "t_end", t(end));
end

function [pos_ecef, vel_ecef] = lla_to_ecef_and_velocity(lat_deg, lon_deg, alt, t)
    phi = deg2rad(lat_deg);
    lambda = deg2rad(lon_deg);

    a_wgs = 6378137.0;
    e_wgs = 8.1819190842622e-2;

    N = a_wgs ./ sqrt(1 - e_wgs^2 .* sin(phi).^2);

    x = (N + alt) .* cos(phi) .* cos(lambda);
    y = (N + alt) .* cos(phi) .* sin(lambda);
    z = (N .* (1 - e_wgs^2) + alt) .* sin(phi);

    vx = gradient(x, t);
    vy = gradient(y, t);
    vz = gradient(z, t);

    if numel(t) >= 2
        vx(1) = vx(2);
        vy(1) = vy(2);
        vz(1) = vz(2);
        vx(end) = vx(end-1);
        vy(end) = vy(end-1);
        vz(end) = vz(end-1);
    end

    pos_ecef = [x.'; y.'; z.'];
    vel_ecef = [vx.'; vy.'; vz.'];
end

function [pos_eci, vel_eci] = ecef_history_to_eci(pos_ecef, vel_ecef, t)
    omega_E = 7.2921159e-5;
    omega_vec = [0; 0; omega_E];
    n_samples = numel(t);
    pos_eci = zeros(3, n_samples);
    vel_eci = zeros(3, n_samples);

    for k = 1:n_samples
        theta = omega_E * t(k);
        C_ecef_to_eci = [ cos(theta), -sin(theta), 0; ...
                          sin(theta),  cos(theta), 0; ...
                          0,           0,          1];

        pos_eci(:, k) = C_ecef_to_eci * pos_ecef(:, k);
        vel_eci(:, k) = C_ecef_to_eci * (vel_ecef(:, k) + cross(omega_vec, pos_ecef(:, k)));
    end
end

function y = local_moving_rms(x, window)
    x = x(:);
    n = numel(x);
    y = zeros(n, 1);
    half_window = floor(window / 2);

    for k = 1:n
        i0 = max(1, k - half_window);
        i1 = min(n, k + half_window);
        segment = x(i0:i1);
        segment = segment(isfinite(segment));

        if isempty(segment)
            y(k) = NaN;
        else
            y(k) = sqrt(mean(segment.^2));
        end
    end
end

function value = robust_rms(x)
    x = x(:);
    x = x(isfinite(x));

    if isempty(x)
        value = NaN;
    else
        value = sqrt(mean(x.^2));
    end
end

function value = robust_median(x)
    x = x(:);
    x = x(isfinite(x));

    if isempty(x)
        value = NaN;
    else
        value = median(abs(x));
    end
end

function A = symmetrize_psd(A)
    A = 0.5 * (A + A.');
    [V, D] = eig(A);
    d = max(real(diag(D)), 1e-12);
    A = V * diag(d) * V.';
    A = 0.5 * (A + A.');
end

function [gnssSensor, profile] = prepare_gnss_sensor_workspace(varargin)
%PREPARE_GNSS_SENSOR_WORKSPACE Prepare plant-driven GNSS sensor inputs.
%
% This creates time-varying GNSS noise and measurement covariance from the
% real GNSS dataset. The Simulink GNSS subsystem samples the plant state every
% gnss_sample_time seconds and adds these noise signals.

    p = inputParser;
    p.addParameter("Filename", "cov_perturb_POS_s6a_Y24D011_fixed.dat");
    p.addParameter("StartDateJulian", juliandate(datetime(2024, 1, 11)));
    p.addParameter("Seed", 42);
    p.addParameter("StopTime", []);
    p.addParameter("AssignToBase", false);
    p.addParameter("CovarianceWindow", 31);
    p.addParameter("SigmaFloorPosition", 0.05);
    p.addParameter("SigmaFloorVelocity", 0.005);
    p.parse(varargin{:});
    opts = p.Results;

    profile = load_gnss_sensor_profile( ...
        opts.Filename, ...
        opts.StartDateJulian, ...
        "CovarianceWindow", opts.CovarianceWindow, ...
        "SigmaFloorPosition", opts.SigmaFloorPosition, ...
        "SigmaFloorVelocity", opts.SigmaFloorVelocity);

    t = profile.t(:);
    desired_sample_time = 3;
    t = (0:desired_sample_time:t(end)).';

    if ~isempty(opts.StopTime)
        keep = t <= (double(opts.StopTime) + profile.sample_time);

        if any(keep)
            t = t(keep);
        end
    end

    n_samples = numel(t);

    rng(opts.Seed, "twister");

    pos_noise_eci = zeros(n_samples, 3);
    vel_noise_eci = zeros(n_samples, 3);
    R_state_eci_flat = zeros(n_samples, 36);

    for k = 1:n_samples
        %source_index = k;
        source_index = min(numel(profile.t), round(t(k) / profile.sample_time) + 1);

        R_pos = profile.R_state_eci(1:3, 1:3, source_index);
        R_vel = profile.R_state_eci(4:6, 4:6, source_index);

        pos_noise_eci(k, :) = (sqrtm_psd(R_pos) * randn(3, 1)).';
        vel_noise_eci(k, :) = (sqrtm_psd(R_vel) * randn(3, 1)).';
        R_state_eci_flat(k, :) = profile.R_state_eci_flat(source_index, :);
    end

    ts_gnss_pos_noise_eci = timeseries(pos_noise_eci, t);
    ts_gnss_pos_noise_eci.Name = "GNSS_PositionNoise_ECI";
    ts_gnss_pos_noise_eci.DataInfo.Units = "m";
    ts_gnss_pos_noise_eci = setinterpmethod(ts_gnss_pos_noise_eci, "zoh");

    ts_gnss_vel_noise_eci = timeseries(vel_noise_eci, t);
    ts_gnss_vel_noise_eci.Name = "GNSS_VelocityNoise_ECI";
    ts_gnss_vel_noise_eci.DataInfo.Units = "m/s";
    ts_gnss_vel_noise_eci = setinterpmethod(ts_gnss_vel_noise_eci, "zoh");

    ts_gnss_R_state_eci_flat = timeseries(R_state_eci_flat, t);
    ts_gnss_R_state_eci_flat.Name = "GNSS_MeasurementCovariance_State_ECI_flat36";
    ts_gnss_R_state_eci_flat.DataInfo.Units = "m2_and_m2s2";
    ts_gnss_R_state_eci_flat = setinterpmethod(ts_gnss_R_state_eci_flat, "zoh");

    ts_gnss_valid = timeseries(ones(n_samples, 1), t);
    ts_gnss_valid.Name = "GNSS_ValidFlag";
    ts_gnss_valid = setinterpmethod(ts_gnss_valid, "zoh");

    gnssSensor = struct();
    gnssSensor.mode = "plant_sensor";
    gnssSensor.sample_time = desired_sample_time;
    %gnssSensor.sample_time = profile.sample_time;
    gnssSensor.seed = opts.Seed;
    gnssSensor.t = t;
    gnssSensor.pos_noise_eci = pos_noise_eci;
    gnssSensor.vel_noise_eci = vel_noise_eci;
    gnssSensor.R_state_eci_flat = R_state_eci_flat;
    gnssSensor.ts_pos_noise_eci = ts_gnss_pos_noise_eci;
    gnssSensor.ts_vel_noise_eci = ts_gnss_vel_noise_eci;
    gnssSensor.ts_R_state_eci_flat = ts_gnss_R_state_eci_flat;
    gnssSensor.ts_valid = ts_gnss_valid;
    gnssSensor.profile_meta = profile.meta;

    if opts.AssignToBase
        assignin("base", "gnss_sensor_mode", gnssSensor.mode);
        assignin("base", "gnss_sample_time", gnssSensor.sample_time);
        assignin("base", "gnssSensor", gnssSensor);
        assignin("base", "gnssProfile", profile);
        assignin("base", "ts_gnss_pos_noise_eci", ts_gnss_pos_noise_eci);
        assignin("base", "ts_gnss_vel_noise_eci", ts_gnss_vel_noise_eci);
        assignin("base", "ts_gnss_R_state_eci_flat", ts_gnss_R_state_eci_flat);
        assignin("base", "ts_gnss_valid", ts_gnss_valid);

        % Workspace aliases kept for Simulink blocks that read these names.
        assignin("base", "ts_pos", profile.ts_pos_eci);
        assignin("base", "ts_vel", profile.ts_vel_eci);
        assignin("base", "ts_Q", profile.ts_R_state_eci_flat);
    end
end

function S = sqrtm_psd(A)
    A = 0.5 * (A + A.');
    [V, D] = eig(A);
    d = max(real(diag(D)), 0);
    S = V * diag(sqrt(d));
end

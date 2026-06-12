function [ts_pos_gnss, ts_vel_gnss, ts_R_gnss, info] = generate_gnss_measurements_from_truth_history(t_truth, x_truth, gnssSensor)
%GENERATE_GNSS_MEASUREMENTS_FROM_TRUTH_HISTORY Offline GNSS sensor.
%
% Use this helper for standalone tests: provide a true state history and the
% gnssSensor struct returned by prepare_gnss_sensor_workspace. The
% function samples truth at GNSS epochs and adds the prepared realistic noise.

    t_truth = t_truth(:);

    if size(x_truth, 1) == 6
        x_truth = x_truth.';
    end

    if size(x_truth, 2) ~= 6
        error("x_truth must be N-by-6 or 6-by-N.");
    end

    t_gnss = gnssSensor.t(:);
    keep = t_gnss >= t_truth(1) & t_gnss <= t_truth(end);
    t_gnss = t_gnss(keep);

    if isempty(t_gnss)
        error("No GNSS epochs fall inside the truth history time span.");
    end

    pos_true = interp1(t_truth, x_truth(:, 1:3), t_gnss, "linear");
    vel_true = interp1(t_truth, x_truth(:, 4:6), t_gnss, "linear");

    pos_noise = gnssSensor.pos_noise_eci(keep, :);
    vel_noise = gnssSensor.vel_noise_eci(keep, :);
    R_flat = gnssSensor.R_state_eci_flat(keep, :);

    ts_pos_gnss = timeseries(pos_true + pos_noise, t_gnss);
    ts_pos_gnss.Name = "GNSS_PositionMeasurement_ECI";
    ts_pos_gnss.DataInfo.Units = "m";
    ts_pos_gnss = setinterpmethod(ts_pos_gnss, "zoh");

    ts_vel_gnss = timeseries(vel_true + vel_noise, t_gnss);
    ts_vel_gnss.Name = "GNSS_VelocityMeasurement_ECI";
    ts_vel_gnss.DataInfo.Units = "m/s";
    ts_vel_gnss = setinterpmethod(ts_vel_gnss, "zoh");

    ts_R_gnss = timeseries(R_flat, t_gnss);
    ts_R_gnss.Name = "GNSS_MeasurementCovariance_State_ECI_flat36";
    ts_R_gnss.DataInfo.Units = "m2_and_m2s2";
    ts_R_gnss = setinterpmethod(ts_R_gnss, "zoh");

    info = struct();
    info.sample_time = gnssSensor.sample_time;
    info.n_samples = numel(t_gnss);
    info.t_start = t_gnss(1);
    info.t_end = t_gnss(end);
end

function report = generate_gnss_validation_figures(outputDir, durationSeconds)
%GENERATE_GNSS_VALIDATION_FIGURES Build presentation-ready GNSS plots.
%
% The demo propagates a truth trajectory with the local dynamics function and
% then applies the plant-driven GNSS sensor profile. This demonstrates that
% the GNSS measurement is no longer a fixed position replay.

    if nargin < 1 || strlength(string(outputDir)) == 0
        outputDir = "figures/gnss_sensor_validation";
    end

    if nargin < 2 || isempty(durationSeconds)
        durationSeconds = 900;
    end

    outputDir = string(outputDir);

    if ~isfolder(outputDir)
        mkdir(outputDir);
    end

    [gnssSensor, profile] = prepare_gnss_sensor_workspace( ...
        "Filename", "cov_perturb_POS_s6a_Y24D011_fixed.dat", ...
        "Seed", 42, ...
        "StopTime", durationSeconds, ...
        "AssignToBase", false);

    x0 = profile.initial_state_eci(:);
    t_truth = (0:1:durationSeconds).';
    x_truth = propagate_truth_with_local_dynamics(x0, t_truth);

    [ts_pos_gnss, ts_vel_gnss, ts_R_gnss, info] = ...
        generate_gnss_measurements_from_truth_history(t_truth, x_truth, gnssSensor);

    t_gnss = ts_pos_gnss.Time(:);
    pos_true_at_gnss = interp1(t_truth, x_truth(:, 1:3), t_gnss, "linear");
    vel_true_at_gnss = interp1(t_truth, x_truth(:, 4:6), t_gnss, "linear");

    pos_error = ts_pos_gnss.Data - pos_true_at_gnss;
    vel_error = ts_vel_gnss.Data - vel_true_at_gnss;

    R_flat = ts_R_gnss.Data;
    sigma_pos_eci = zeros(info.n_samples, 3);
    sigma_vel_eci = zeros(info.n_samples, 3);
    nis_pos = zeros(info.n_samples, 1);

    for k = 1:info.n_samples
        R_state = reshape(R_flat(k, :), 6, 6);
        R_pos = 0.5 * (R_state(1:3, 1:3) + R_state(1:3, 1:3).');
        R_vel = 0.5 * (R_state(4:6, 4:6) + R_state(4:6, 4:6).');
        sigma_pos_eci(k, :) = sqrt(max(diag(R_pos), 0)).';
        sigma_vel_eci(k, :) = sqrt(max(diag(R_vel), 0)).';
        nis_pos(k) = pos_error(k, :) * (R_pos \ pos_error(k, :).');
    end

    make_position_error_figure(outputDir, t_gnss, pos_error, sigma_pos_eci, nis_pos);
    make_velocity_error_figure(outputDir, t_gnss, vel_error, sigma_vel_eci);
    make_quality_profile_figure(outputDir, profile);
    make_orbit_measurement_figure(outputDir, t_truth, x_truth, t_gnss, ts_pos_gnss.Data);

    report = struct();
    report.outputDir = outputDir;
    report.durationSeconds = durationSeconds;
    report.sampleTimeSeconds = gnssSensor.sample_time;
    report.nMeasurements = info.n_samples;
    report.positionRmsM = sqrt(mean(pos_error.^2, 1));
    report.positionNormRmsM = sqrt(mean(sum(pos_error.^2, 2)));
    report.velocityRmsMps = sqrt(mean(vel_error.^2, 1));
    report.velocityNormRmsMps = sqrt(mean(sum(vel_error.^2, 2)));
    report.nisPosMean = mean(nis_pos);
    report.nisPosP99Threshold = 11.345;
    report.nisPosBelowP99Fraction = mean(nis_pos <= report.nisPosP99Threshold);
    report.files = [ ...
        outputDir + "/gnss_position_error_3sigma.png"; ...
        outputDir + "/gnss_velocity_error_3sigma.png"; ...
        outputDir + "/gnss_quality_profile.png"; ...
        outputDir + "/gnss_orbit_measurements.png"];

    save(outputDir + "/gnss_validation_report.mat", "report", "t_truth", "x_truth", ...
        "t_gnss", "pos_error", "vel_error", "sigma_pos_eci", "sigma_vel_eci", "nis_pos");

    fprintf("\nGNSS validation figures written to %s\n", outputDir);
    fprintf("Duration               : %.1f s\n", durationSeconds);
    fprintf("GNSS sample time       : %.3f s\n", report.sampleTimeSeconds);
    fprintf("GNSS measurements      : %d\n", report.nMeasurements);
    fprintf("Position RMS [x y z]   : %.3f %.3f %.3f m\n", report.positionRmsM);
    fprintf("Position norm RMS      : %.3f m\n", report.positionNormRmsM);
    fprintf("Velocity norm RMS      : %.4f m/s\n", report.velocityNormRmsMps);
    fprintf("NIS pos mean           : %.3f\n", report.nisPosMean);
    fprintf("NIS below 99%% threshold: %.1f %%\n", 100 * report.nisPosBelowP99Fraction);
end

function x_truth = propagate_truth_with_local_dynamics(x0, t_truth)
    x_truth = zeros(numel(t_truth), 6);
    x_truth(1, :) = x0(:).';
    u_zero = zeros(3, 1);

    for k = 2:numel(t_truth)
        x_truth(k, :) = myStateTransitionFcn(x_truth(k-1, :).', u_zero).';
    end
end

function make_position_error_figure(outputDir, t, err, sigma, nis)
    fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 820]);
    tiledlayout(4, 1, "TileSpacing", "compact", "Padding", "compact");
    labels = ["ECI X", "ECI Y", "ECI Z"];

    for i = 1:3
        nexttile;
        hold on;
        fill_between(t, 3*sigma(:, i), -3*sigma(:, i), [0.88, 0.93, 1.00]);
        plot(t, err(:, i), "Color", [0.05, 0.22, 0.42], "LineWidth", 1.2);
        plot(t, 3*sigma(:, i), "--", "Color", [0.20, 0.45, 0.70], "LineWidth", 0.8);
        plot(t, -3*sigma(:, i), "--", "Color", [0.20, 0.45, 0.70], "LineWidth", 0.8);
        grid on;
        ylabel(labels(i) + " [m]");

        if i == 1
            title("Simulated GNSS position error from plant truth with 3-sigma covariance envelope");
        end
    end

    nexttile;
    hold on;
    plot(t, nis, "Color", [0.50, 0.18, 0.12], "LineWidth", 1.2);
    yline(11.345, "--", "99% chi-square threshold, 3 DoF", "Color", [0.25, 0.25, 0.25]);
    grid on;
    xlabel("Time [s]");
    ylabel("Position NIS [-]");
    exportgraphics(fig, outputDir + "/gnss_position_error_3sigma.png", "Resolution", 180);
    close(fig);
end

function make_velocity_error_figure(outputDir, t, err, sigma)
    fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 680]);
    tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");
    labels = ["ECI Vx", "ECI Vy", "ECI Vz"];

    for i = 1:3
        nexttile;
        hold on;
        fill_between(t, 3*sigma(:, i), -3*sigma(:, i), [0.91, 0.96, 0.91]);
        plot(t, err(:, i), "Color", [0.05, 0.35, 0.16], "LineWidth", 1.2);
        plot(t, 3*sigma(:, i), "--", "Color", [0.22, 0.52, 0.28], "LineWidth", 0.8);
        plot(t, -3*sigma(:, i), "--", "Color", [0.22, 0.52, 0.28], "LineWidth", 0.8);
        grid on;
        ylabel(labels(i) + " [m/s]");

        if i == 1
            title("Simulated GNSS velocity error with covariance envelope");
        end

        if i == 3
            xlabel("Time [s]");
        end
    end

    exportgraphics(fig, outputDir + "/gnss_velocity_error_3sigma.png", "Resolution", 180);
    close(fig);
end

function make_quality_profile_figure(outputDir, profile)
    fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 780]);
    tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

    t_hours = profile.t(:) / 3600;

    nexttile;
    plot(t_hours, profile.sigma_neu(:, 1), "LineWidth", 1.1); hold on;
    plot(t_hours, profile.sigma_neu(:, 2), "LineWidth", 1.1);
    plot(t_hours, profile.sigma_neu(:, 3), "LineWidth", 1.1);
    grid on;
    ylabel("sigma [m]");
    title("GNSS quality profile extracted from dataset");
    legend(["North", "East", "Up"], "Location", "best");

    nexttile;
    stairs(t_hours, profile.nsv, "Color", [0.10, 0.31, 0.52], "LineWidth", 1.1); hold on;
    stairs(t_hours, profile.nsv_visible, "Color", [0.62, 0.31, 0.13], "LineWidth", 1.1);
    grid on;
    ylabel("satellites [-]");
    legend(["Used", "Visible"], "Location", "best");

    nexttile;
    plot(t_hours, profile.hdop, "LineWidth", 1.1); hold on;
    plot(t_hours, profile.vdop, "LineWidth", 1.1);
    plot(t_hours, profile.pdop, "LineWidth", 1.1);
    grid on;
    xlabel("Dataset time [h]");
    ylabel("DOP [-]");
    legend(["HDOP", "VDOP", "PDOP"], "Location", "best");

    exportgraphics(fig, outputDir + "/gnss_quality_profile.png", "Resolution", 180);
    close(fig);
end

function make_orbit_measurement_figure(outputDir, t_truth, x_truth, t_gnss, z_pos)
    fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1050, 780]);
    plot3(x_truth(:, 1)/1000, x_truth(:, 2)/1000, x_truth(:, 3)/1000, ...
        "Color", [0.06, 0.18, 0.35], "LineWidth", 1.4);
    hold on;
    scatter3(z_pos(:, 1)/1000, z_pos(:, 2)/1000, z_pos(:, 3)/1000, ...
        20, t_gnss, "filled");
    axis equal;
    grid on;
    xlabel("ECI X [km]");
    ylabel("ECI Y [km]");
    zlabel("ECI Z [km]");
    title("Plant truth trajectory sampled by simulated GNSS every 10 s");
    cb = colorbar;
    cb.Label.String = "GNSS measurement time [s]";
    legend(["Plant truth", "Simulated GNSS measurements"], "Location", "best");
    view(45, 25);
    exportgraphics(fig, outputDir + "/gnss_orbit_measurements.png", "Resolution", 180);
    close(fig);
end

function fill_between(x, y_upper, y_lower, color)
    x = x(:);
    y_upper = y_upper(:);
    y_lower = y_lower(:);
    fill([x; flipud(x)], [y_upper; flipud(y_lower)], color, ...
        "LineStyle", "none", "FaceAlpha", 0.65);
end

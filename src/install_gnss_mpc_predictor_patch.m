function install_gnss_mpc_predictor_patch(modelName)
%INSTALL_GNSS_MPC_PREDICTOR_PATCH Add a GNSS-only state predictor before MPC.
%
% The raw simulated GNSS output is sampled at gnss_sample_time. Feeding that
% raw zero-order-held state directly to the MPC creates a large time-lag
% error between GNSS samples. This patch inserts:
%
%   raw GNSS position/velocity -> GNSS State Predictor -> MPC Mux
%
% The predictor propagates the last GNSS state to the current simulation
% time using the same central gravity + J2 model family used by the UKF.

    if nargin < 1 || strlength(string(modelName)) == 0
        modelName = "MPCcontrolledSpacecraft_plant_gnss";
    end

    modelName = string(modelName);
    modelFile = modelName + ".slx";

    if ~isfile(modelFile)
        error("Model file not found: %s", modelFile);
    end

    backupFile = modelName + "_before_gnss_predictor_" + datestr(now, "yyyymmdd_HHMMSS") + ".slx";
    copyfile(modelFile, backupFile);
    fprintf("Backed up current model as %s\n", backupFile);

    load_system(modelName);

    gnssPath = find_enabled_gnss_subsystem(modelName);
    predictorPath = modelName + "/GNSS State Predictor";

    if getSimulinkBlockHandle(predictorPath) ~= -1
        delete_block(predictorPath);
    end

    add_block("simulink/Ports & Subsystems/Subsystem", predictorPath, ...
        "Position", [-1660, 490, -1460, 590]);

    rebuild_predictor_subsystem(predictorPath);
    connect_predictor(modelName, gnssPath, predictorPath);

    save_system(modelName);
    fprintf("GNSS State Predictor inserted in %s\n", modelName);
end

function gnssPath = find_enabled_gnss_subsystem(modelName)
    candidates = find_system(modelName, "SearchDepth", 1, "BlockType", "SubSystem");

    for k = 1:numel(candidates)
        candidate = string(candidates{k});
        description = string(get_param(candidate, "Description"));
        ports = get_param(candidate, "Ports");

        if contains(description, "GNSS", "IgnoreCase", true) && ...
           numel(ports) >= 3 && ports(1) >= 2 && ports(2) >= 2
            gnssPath = candidate;
            return;
        end
    end

    error("Could not find the enabled GNSS subsystem in %s.", modelName);
end

function rebuild_predictor_subsystem(predictorPath)
    delete_internal_lines(predictorPath);
    delete_child_blocks(predictorPath);

    add_block("simulink/Sources/In1", predictorPath + "/gnss_position", ...
        "Port", "1", "Position", [35, 70, 65, 90]);
    add_block("simulink/Sources/In1", predictorPath + "/gnss_velocity", ...
        "Port", "2", "Position", [35, 150, 65, 170]);
    add_block("simulink/Sources/Clock", predictorPath + "/Clock", ...
        "Position", [35, 230, 65, 250]);
    add_block("simulink/Sources/Constant", predictorPath + "/GNSS Sample Time", ...
        "Value", "gnss_sample_time", "Position", [25, 300, 95, 325]);

    add_block("simulink/User-Defined Functions/MATLAB Function", ...
        predictorPath + "/Predict GNSS State", ...
        "Position", [160, 95, 310, 245]);

    add_block("simulink/Sinks/Out1", predictorPath + "/position", ...
        "Port", "1", "InitialOutput", "zeros(3,1)", "Position", [395, 105, 425, 125]);
    add_block("simulink/Sinks/Out1", predictorPath + "/velocity", ...
        "Port", "2", "InitialOutput", "zeros(3,1)", "Position", [395, 185, 425, 205]);

    root = sfroot;
    chart = root.find("-isa", "Stateflow.EMChart", ...
        "Path", char(predictorPath + "/Predict GNSS State"));

    if isempty(chart)
        error("Could not find MATLAB Function chart for GNSS predictor.");
    end

    chart.Script = char(strjoin([ ...
        "function [position, velocity] = predict_gnss_state(gnss_position, gnss_velocity, t, sample_time)", ...
        "%#codegen", ...
        "sample_time = max(sample_time, eps);", ...
        "dt = t - floor(t / sample_time) * sample_time;", ...
        "dt = max(dt, 0);", ...
        "x0 = [gnss_position(:); gnss_velocity(:)];", ...
        "if dt <= eps", ...
        "    x = x0;", ...
        "else", ...
        "    x = rk4_orbit_step(x0, dt);", ...
        "end", ...
        "position = x(1:3);", ...
        "velocity = x(4:6);", ...
        "end", ...
        "", ...
        "function x_next = rk4_orbit_step(x, dt)", ...
        "k1 = orbit_derivative(x);", ...
        "k2 = orbit_derivative(x + 0.5 * dt * k1);", ...
        "k3 = orbit_derivative(x + 0.5 * dt * k2);", ...
        "k4 = orbit_derivative(x + dt * k3);", ...
        "x_next = x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);", ...
        "end", ...
        "", ...
        "function x_dot = orbit_derivative(x)", ...
        "mu_earth = 3.986004418e14;", ...
        "R_earth = 6378137.0;", ...
        "J2 = 1.08262668e-3;", ...
        "r_vec = x(1:3);", ...
        "v_vec = x(4:6);", ...
        "r = norm(r_vec);", ...
        "z = r_vec(3);", ...
        "a_central = -(mu_earth / r^3) * r_vec;", ...
        "zx = z / r;", ...
        "factor = 1.5 * J2 * mu_earth * R_earth^2 / r^5;", ...
        "a_j2 = factor * [r_vec(1) * (5*zx^2 - 1); r_vec(2) * (5*zx^2 - 1); r_vec(3) * (5*zx^2 - 3)];", ...
        "x_dot = [v_vec; a_central + a_j2];", ...
        "end"], newline));

    add_line_safe(predictorPath, "gnss_position/1", "Predict GNSS State/1");
    add_line_safe(predictorPath, "gnss_velocity/1", "Predict GNSS State/2");
    add_line_safe(predictorPath, "Clock/1", "Predict GNSS State/3");
    add_line_safe(predictorPath, "GNSS Sample Time/1", "Predict GNSS State/4");
    add_line_safe(predictorPath, "Predict GNSS State/1", "position/1");
    add_line_safe(predictorPath, "Predict GNSS State/2", "velocity/1");
end

function connect_predictor(modelName, gnssPath, predictorPath)
    muxPath = modelName + "/Mux";

    delete_input_line(muxPath, 1);
    delete_input_line(muxPath, 2);

    gnssName = string(get_param(gnssPath, "Name"));
    predictorName = string(get_param(predictorPath, "Name"));

    add_line_safe(modelName, gnssName + "/1", predictorName + "/1");
    add_line_safe(modelName, gnssName + "/2", predictorName + "/2");
    add_line_safe(modelName, predictorName + "/1", "Mux/1");
    add_line_safe(modelName, predictorName + "/2", "Mux/2");

    reconnect_optional_error_scopes(modelName, gnssName);
end

function reconnect_optional_error_scopes(modelName, gnssName)
    if getSimulinkBlockHandle(modelName + "/Sum1") ~= -1
        add_line_safe(modelName, "Spacecraft Dynamics/1", "Sum1/1");
        add_line_safe(modelName, gnssName + "/1", "Sum1/2");
    end

    if getSimulinkBlockHandle(modelName + "/Sum2") ~= -1
        add_line_safe(modelName, "Spacecraft Dynamics/2", "Sum2/1");
        add_line_safe(modelName, gnssName + "/2", "Sum2/2");
    end
end

function delete_input_line(blockPath, portNumber)
    ports = get_param(blockPath, "PortHandles");
    line = get_param(ports.Inport(portNumber), "Line");

    if line ~= -1
        delete_line(line);
    end
end

function delete_internal_lines(systemPath)
    lines = find_system(systemPath, "FindAll", "on", "SearchDepth", 1, "Type", "line");

    for k = 1:numel(lines)
        try
            delete_line(lines(k));
        catch
        end
    end
end

function delete_child_blocks(systemPath)
    blocks = find_system(systemPath, "SearchDepth", 1, "Type", "Block");

    for k = 1:numel(blocks)
        blockPath = string(blocks{k});

        if blockPath ~= string(systemPath)
            delete_block(blockPath);
        end
    end
end

function add_line_safe(systemPath, src, dst)
    try
        add_line(systemPath, src, dst, "autorouting", "on");
    catch ME
        if ~contains(ME.message, "already connected", "IgnoreCase", true) && ...
           ~contains(ME.message, "destination port already has a line connection", "IgnoreCase", true) && ...
           ~contains(ME.message, "valid connection", "IgnoreCase", true)
            rethrow(ME);
        end
    end
end

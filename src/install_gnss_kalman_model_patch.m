function patchedModel = install_gnss_kalman_model_patch(sourceModel, patchedModel)
%INSTALL_GNSS_KALMAN_MODEL_PATCH Route simulated GNSS through the UKF.
%
% The resulting model uses:
%
%   plant -> simulated GNSS measurements -> UKF measurement input
%   UKF estimated state -> MPC
%
% This avoids feeding raw sampled GNSS directly to the MPC.

    if nargin < 1 || strlength(string(sourceModel)) == 0
        sourceModel = "MPCcontrolledSpacecraft_plant_gnss";
    end

    if nargin < 2 || strlength(string(patchedModel)) == 0
        patchedModel = "MPCcontrolledSpacecraft_plant_gnss_kalman";
    end

    sourceModel = string(sourceModel);
    patchedModel = string(patchedModel);

    if ~isfile(sourceModel + ".slx")
        error("Source model file not found: %s.slx", sourceModel);
    end

    if isfile(patchedModel + ".slx")
        backupFile = patchedModel + "_backup_" + datestr(now, "yyyymmdd_HHMMSS") + ".slx";
        copyfile(patchedModel + ".slx", backupFile);
        fprintf("Existing patched model backed up as %s\n", backupFile);
    end

    load_system(sourceModel);
    save_system(sourceModel, patchedModel);
    load_system(patchedModel);

    gnssPath = find_enabled_gnss_subsystem(patchedModel);
    kalmanPath = patchedModel + "/KALMAN FILTER";

    configure_gnss_enable(patchedModel, gnssPath);
    configure_kalman_for_gnss(kalmanPath);
    connect_gnss_to_kalman(patchedModel, gnssPath);
    connect_kalman_to_mpc(patchedModel);
    % Keep the GNSS State Predictor as a measurement-rate adapter for the UKF.

    save_system(patchedModel);
    fprintf("GNSS-through-Kalman model saved as %s.slx\n", patchedModel);
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

    error("Could not find enabled GNSS subsystem in %s.", modelName);
end

function configure_gnss_enable(modelName, gnssPath)
    ports = get_param(gnssPath, "PortHandles");

    if isempty(ports.Enable)
        return;
    end

    enableLine = get_param(ports.Enable(1), "Line");

    if enableLine ~= -1
        delete_line(enableLine);
    end

    constantPath = modelName + "/GNSS Measurement Enable";

    if getSimulinkBlockHandle(constantPath) == -1
        add_block("simulink/Sources/Constant", constantPath, ...
            "Value", "1", ...
            "Position", [-2100, 455, -2060, 485]);
    else
        set_param(constantPath, "Value", "1");
    end

    constantPorts = get_param(constantPath, "PortHandles");
    add_line(modelName, constantPorts.Outport(1), ports.Enable(1), "autorouting", "on");
end

function configure_kalman_for_gnss(kalmanPath)
    gnssPos = find_or_add_port_block(kalmanPath, "GNSS_Position_ECI", ...
        "simulink/Sources/In1", "Inport", 4);
    gnssVel = find_or_add_port_block(kalmanPath, "GNSS_Velocity_ECI", ...
        "simulink/Sources/In1", "Inport", 5);
    set_param(gnssPos, "Position", [-30, 335, 0, 355]);
    set_param(gnssVel, "Position", [-30, 400, 0, 420]);

    muxPath = kalmanPath + "/GNSS State Measurement";

    if getSimulinkBlockHandle(muxPath) ~= -1
        delete_block(muxPath);
    end

    add_block("simulink/Signal Routing/Mux", muxPath, ...
        "Inputs", "2", ...
        "Position", [65, 340, 95, 415]);

    measurementHold = find_measurement_hold(kalmanPath);

    set_param(measurementHold, "SampleTime", "Ts");
    delete_input_line(measurementHold, 1);

    add_line_safe(kalmanPath, "GNSS_Position_ECI/1", "GNSS State Measurement/1");
    add_line_safe(kalmanPath, "GNSS_Velocity_ECI/1", "GNSS State Measurement/2");
    add_line_by_handles(kalmanPath, muxPath, 1, measurementHold, 1);

    ukfBlocks = find_system(kalmanPath, ...
        "SearchDepth", 1, ...
        "Name", "Unscented Kalman Filter_");

    if isempty(ukfBlocks)
        error("Could not find the Unscented Kalman Filter block.");
    end

    ukfPath = string(ukfBlocks{1});
    set_param(ukfPath, ...
        "MeasurementFcn1", "myGnssStateMeasurementFcn", ...
        "MeasurementNoise1", "R_gnss_state_matrix", ...
        "MeasurementFcn1SampleTime", "Ts");
end

function measurementHold = find_measurement_hold(kalmanPath)
    holds = find_system(kalmanPath, ...
        "SearchDepth", 1, ...
        "BlockType", "ZeroOrderHold");

    for k = 1:numel(holds)
        holdPath = string(holds{k});
        name = string(get_param(holdPath, "Name"));

        if ~endsWith(name, "1")
            measurementHold = holdPath;
            return;
        end
    end

    error("Could not find Kalman measurement Zero-Order Hold.");
end

function connect_gnss_to_kalman(modelName, gnssPath)
    kalmanName = "KALMAN FILTER";
    predictorPath = modelName + "/GNSS State Predictor";

    if getSimulinkBlockHandle(predictorPath) ~= -1
        sourceName = "GNSS State Predictor";
    else
        sourceName = string(get_param(gnssPath, "Name"));
    end

    delete_input_line(modelName + "/" + kalmanName, 4);
    delete_input_line(modelName + "/" + kalmanName, 5);

    add_line_safe(modelName, sourceName + "/1", kalmanName + "/4");
    add_line_safe(modelName, sourceName + "/2", kalmanName + "/5");
end

function connect_kalman_to_mpc(modelName)
    muxPath = modelName + "/Mux";
    delete_input_line(muxPath, 1);
    delete_input_line(muxPath, 2);
    add_line_safe(modelName, "KALMAN FILTER/1", "Mux/1");
    add_line_safe(modelName, "KALMAN FILTER/2", "Mux/2");
end

function blockPath = find_or_add_port_block(systemPath, blockName, libraryBlock, blockType, portNumber)
    blocks = find_system(systemPath, "SearchDepth", 1, "BlockType", blockType);
    blockPath = "";

    for k = 1:numel(blocks)
        candidate = string(blocks{k});

        if get_port_number(candidate) == portNumber
            blockPath = candidate;
            break;
        end
    end

    if strlength(blockPath) == 0
        blockPath = systemPath + "/" + blockName;
        add_block(libraryBlock, blockPath, "MakeNameUnique", "off");
    else
        currentName = string(get_param(blockPath, "Name"));

        if currentName ~= blockName
            targetPath = systemPath + "/" + blockName;

            if getSimulinkBlockHandle(targetPath) ~= -1 && targetPath ~= blockPath
                delete_block(targetPath);
            end

            set_param(blockPath, "Name", blockName);
            blockPath = targetPath;
        end
    end

    set_param(blockPath, "Port", string(portNumber));
end

function portNumber = get_port_number(blockPath)
    portText = string(get_param(blockPath, "Port"));
    portNumber = str2double(portText);

    if isnan(portNumber)
        portNumber = 1;
    end
end

function delete_input_line(blockPath, portNumber)
    if getSimulinkBlockHandle(blockPath) == -1
        return;
    end

    ports = get_param(blockPath, "PortHandles");

    if numel(ports.Inport) < portNumber
        return;
    end

    line = get_param(ports.Inport(portNumber), "Line");

    if line ~= -1
        delete_line(line);
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

function add_line_by_handles(systemPath, srcBlock, srcPort, dstBlock, dstPort)
    srcPorts = get_param(srcBlock, "PortHandles");
    dstPorts = get_param(dstBlock, "PortHandles");

    try
        add_line(systemPath, srcPorts.Outport(srcPort), dstPorts.Inport(dstPort), "autorouting", "on");
    catch ME
        if ~contains(ME.message, "already connected", "IgnoreCase", true) && ...
           ~contains(ME.message, "destination port already has a line connection", "IgnoreCase", true) && ...
           ~contains(ME.message, "valid connection", "IgnoreCase", true)
            rethrow(ME);
        end
    end
end

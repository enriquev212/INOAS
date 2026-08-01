function patchedModel = install_gnss_sensor_model_patch(sourceModel, patchedModel, includeCovarianceOutput)
%INSTALL_GNSS_SENSOR_MODEL_PATCH Create a plant-driven GNSS Simulink model.
%
% The generated model keeps the original model intact. It rebuilds the
% GNSS subsystem so that GNSS is:
%
%   z_pos = sample_hold(true_position, gnss_sample_time) + pos_noise
%   z_vel = sample_hold(true_velocity, gnss_sample_time) + vel_noise
%
% The noise and covariance signals are prepared by
% prepare_gnss_sensor_workspace.m.

    if nargin < 1 || strlength(string(sourceModel)) == 0
        sourceModel = "GNSS_MPCcontrolledSpacecraft";
    end

    if nargin < 2 || strlength(string(patchedModel)) == 0
        patchedModel = string(sourceModel) + "_plant_gnss";
    end

    if nargin < 3 || isempty(includeCovarianceOutput)
        includeCovarianceOutput = true;
    end

    sourceModel = string(sourceModel);
    patchedModel = string(patchedModel);
    includeCovarianceOutput = logical(includeCovarianceOutput);

    load_system(sourceModel);

    patchedFile = patchedModel + ".slx";

    if isfile(patchedFile)
        backupFile = patchedModel + "_backup_" + datestr(now, "yyyymmdd_HHMMSS") + ".slx";
        copyfile(patchedFile, backupFile);
        fprintf("Existing patched model backed up as %s\n", backupFile);
    end

    save_system(sourceModel, patchedModel);
    load_system(patchedModel);

    gnssPath = find_gnss_subsystem(patchedModel);
    rebuild_gnss_subsystem(gnssPath, includeCovarianceOutput);
    connect_plant_to_gnss(patchedModel, gnssPath);
    patch_mpc_wrapper_fixed_sizes(patchedModel, 3, 50);

    save_system(patchedModel);
    fprintf("Plant-driven GNSS model saved as %s.slx\n", patchedModel);
end

function rebuild_gnss_subsystem(gnssPath, includeCovarianceOutput)
    delete_internal_lines(gnssPath);
    delete_non_port_blocks(gnssPath);

    enablePath = find_or_add_block(gnssPath, "Enable", "simulink/Ports & Subsystems/Enable", "EnablePort");
    set_param(enablePath, "Position", [240, -25, 260, -5]);

    truePos = find_or_add_port_block(gnssPath, "true_position", "simulink/Sources/In1", "Inport", 1);
    trueVel = find_or_add_port_block(gnssPath, "true_velocity", "simulink/Sources/In1", "Inport", 2);
    delete_port_blocks_above(gnssPath, "Inport", 2);
    set_param(truePos, "Port", "1", "Position", [35, 70, 65, 90]);
    set_param(trueVel, "Port", "2", "Position", [35, 190, 65, 210]);

    zohPos = add_block_unique("simulink/Discrete/Zero-Order Hold", gnssPath + "/GNSS Position Sample Hold");
    zohVel = add_block_unique("simulink/Discrete/Zero-Order Hold", gnssPath + "/GNSS Velocity Sample Hold");
    set_param(zohPos, "SampleTime", "gnss_sample_time", "Position", [105, 62, 150, 98]);
    set_param(zohVel, "SampleTime", "gnss_sample_time", "Position", [105, 182, 150, 218]);

    posNoise = add_block_unique("simulink/Sources/From Workspace", gnssPath + "/GNSS Position Noise");
    velNoise = add_block_unique("simulink/Sources/From Workspace", gnssPath + "/GNSS Velocity Noise");
    set_param(posNoise, "VariableName", "ts_gnss_pos_noise_eci", "SampleTime", "Ts", "Position", [105, 122, 185, 148]);
    set_param(velNoise, "VariableName", "ts_gnss_vel_noise_eci", "SampleTime", "Ts", "Position", [105, 242, 185, 268]);

    sumPos = add_block_unique("simulink/Math Operations/Sum", gnssPath + "/Add GNSS Position Noise");
    sumVel = add_block_unique("simulink/Math Operations/Sum", gnssPath + "/Add GNSS Velocity Noise");
    set_param(sumPos, "Inputs", "++", "Position", [235, 78, 260, 112]);
    set_param(sumVel, "Inputs", "++", "Position", [235, 198, 260, 232]);

    posOut = find_or_add_port_block(gnssPath, "position", "simulink/Sinks/Out1", "Outport", 1);
    velOut = find_or_add_port_block(gnssPath, "velocity", "simulink/Sinks/Out1", "Outport", 2);
    set_param(posOut, "Port", "1", "InitialOutput", "zeros(3,1)", "Position", [435, 88, 465, 112]);
    set_param(velOut, "Port", "2", "InitialOutput", "zeros(3,1)", "Position", [435, 208, 465, 232]);

    if includeCovarianceOutput
        RState = add_block_unique("simulink/Sources/From Workspace", gnssPath + "/GNSS R State");
        reshapeR = add_block_unique("simulink/Math Operations/Reshape", gnssPath + "/Reshape R State");
        covOut = find_or_add_port_block(gnssPath, "Cov_matrix", "simulink/Sinks/Out1", "Outport", 3);
        set_param(RState, "VariableName", "ts_gnss_R_state_eci_flat", "SampleTime", "Ts", "Position", [105, 330, 195, 356]);
        set_param(reshapeR, ...
            "OutputDimensionality", "Customize", ...
            "OutputDimensions", "[6 6]", ...
            "Position", [255, 328, 295, 352]);
        set_param(covOut, "Port", "3", "InitialOutput", "zeros(6,6)", "Position", [435, 328, 465, 352]);
    else
        delete_port_blocks_above(gnssPath, "Outport", 2);
    end

    add_line_safe(gnssPath, "true_position/1", "GNSS Position Sample Hold/1");
    add_line_safe(gnssPath, "GNSS Position Sample Hold/1", "Add GNSS Position Noise/1");
    add_line_safe(gnssPath, "GNSS Position Noise/1", "Add GNSS Position Noise/2");
    add_line_safe(gnssPath, "Add GNSS Position Noise/1", "position/1");

    add_line_safe(gnssPath, "true_velocity/1", "GNSS Velocity Sample Hold/1");
    add_line_safe(gnssPath, "GNSS Velocity Sample Hold/1", "Add GNSS Velocity Noise/1");
    add_line_safe(gnssPath, "GNSS Velocity Noise/1", "Add GNSS Velocity Noise/2");
    add_line_safe(gnssPath, "Add GNSS Velocity Noise/1", "velocity/1");

    if includeCovarianceOutput
        add_line_safe(gnssPath, "GNSS R State/1", "Reshape R State/1");
        add_line_safe(gnssPath, "Reshape R State/1", "Cov_matrix/1");
    end
end

function gnssPath = find_gnss_subsystem(modelName)
    preferredPath = modelName + "/GNSS Subsystem";

    if getSimulinkBlockHandle(preferredPath) ~= -1
        gnssPath = preferredPath;
        return;
    end

    candidates = find_system(modelName, "SearchDepth", 1, "BlockType", "SubSystem");

    for k = 1:numel(candidates)
        candidate = string(candidates{k});
        fromWorkspaceBlocks = find_system(candidate, ...
            "SearchDepth", 1, ...
            "BlockType", "FromWorkspace");

        variableNames = strings(0, 1);

        for j = 1:numel(fromWorkspaceBlocks)
            variableNames(end+1, 1) = string(get_param(fromWorkspaceBlocks{j}, "VariableName")); %#ok<AGROW>
        end

        if all(ismember(["ts_pos"; "ts_vel"; "ts_Q"], variableNames))
            gnssPath = candidate;
            return;
        end
    end

    for k = 1:numel(candidates)
        candidate = string(candidates{k});
        description = string(get_param(candidate, "Description"));
        ports = get_param(candidate, "Ports");

        if contains(description, "GNSS", "IgnoreCase", true) && ...
           numel(ports) >= 3 && ports(1) >= 2 && ports(2) >= 2 && ports(3) >= 1
            gnssPath = candidate;
            return;
        end
    end

    error("Could not find the GNSS replay subsystem in %s.", modelName);
end

function connect_plant_to_gnss(modelName, gnssPath)
    gnssBlockName = string(get_param(gnssPath, "Name"));
    add_line_safe(modelName, "Spacecraft Dynamics/1", gnssBlockName + "/1");
    add_line_safe(modelName, "Spacecraft Dynamics/2", gnssBlockName + "/2");
end

function patch_mpc_wrapper_fixed_sizes(modelName, m, Np)
    root = sfroot;
    charts = root.find("-isa", "Stateflow.EMChart");

    for k = 1:numel(charts)
        chart = charts(k);

        if ~startsWith(string(chart.Path), modelName + "/")
            continue;
        end

        script = string(chart.Script);

        if ~contains(script, "MPC_INOAS")
            continue;
        end

        script = regexprep(script, "delta_Ulast = zeros\(\d+,1\);", ...
            "delta_Ulast = zeros(" + string(m*Np) + ",1);");
        script = regexprep(script, "slack_opt = zeros\(\d+,1\);", ...
            "slack_opt = zeros(" + string(Np) + ",1);");
        chart.Script = char(script);
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

function delete_non_port_blocks(systemPath)
    blocks = find_system(systemPath, "SearchDepth", 1, "Type", "Block");

    for k = 1:numel(blocks)
        blockPath = string(blocks{k});

        if blockPath == string(systemPath)
            continue;
        end

        blockType = string(get_param(blockPath, "BlockType"));

        if any(blockType == ["Inport", "Outport", "EnablePort"])
            continue;
        end

        delete_block(blockPath);
    end
end

function blockPath = find_or_add_block(systemPath, blockName, libraryBlock, blockType)
    blockPath = systemPath + "/" + blockName;

    matches = find_system(systemPath, "SearchDepth", 1, "Name", blockName, "BlockType", blockType);

    if isempty(matches)
        add_block(libraryBlock, blockPath, "MakeNameUnique", "off");
    else
        blockPath = string(matches{1});
    end
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

        if getSimulinkBlockHandle(blockPath) ~= -1
            delete_block(blockPath);
        end

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

function delete_port_blocks_above(systemPath, blockType, maxPort)
    blocks = find_system(systemPath, "SearchDepth", 1, "BlockType", blockType);

    for k = 1:numel(blocks)
        blockPath = string(blocks{k});

        if get_port_number(blockPath) > maxPort
            delete_block(blockPath);
        end
    end
end

function portNumber = get_port_number(blockPath)
    portText = string(get_param(blockPath, "Port"));
    portNumber = str2double(portText);

    if isnan(portNumber)
        portNumber = 1;
    end
end

function dst = add_block_unique(src, dst)
    if ~isempty(find_system(fileparts_as_simulink(dst), "SearchDepth", 1, "Name", block_name(dst)))
        delete_block(dst);
    end

    add_block(src, dst, "MakeNameUnique", "off");
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

function parent = fileparts_as_simulink(pathText)
    pathText = string(pathText);
    splitPath = split(pathText, "/");
    parent = join(splitPath(1:end-1), "/");
end

function name = block_name(pathText)
    splitPath = split(string(pathText), "/");
    name = splitPath(end);
end

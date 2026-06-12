function apply_kalman_architecture_fix(model)
%APPLY_KALMAN_ARCHITECTURE_FIX Fix GNSS/Kalman selector semantics.
%
% The instrument decision uses:
%   lambda = 1 -> GNSS is selected for the MPC
%   lambda = 0 -> Kalman is selected for the MPC
%
% The UKF correction enable must not be tied to that selector: the Kalman
% branch is an independent navigation source and keeps correcting with its
% own synthetic sensors even when the MPC is currently using GNSS.

if nargin < 1 || strlength(string(model)) == 0
    model = "MPCcontrolledSpacecraft_plant_gnss_kalman_decision";
else
    model = string(model);
end

load_system(model);

gnssBlock = find_system(model, "SearchDepth", 1, "Name", sprintf("Enabled\nSubsystem1"));
if isempty(gnssBlock)
    error("Could not find GNSS enabled subsystem in %s.", model);
end
gnssBlock = string(gnssBlock{1});

kfBlock = model + "/KALMAN FILTER";

% Switch semantics:
% Switch input 1 is selected when lambda > 0.5.
% Therefore input 1 must be GNSS and input 3 must be Kalman.
connect_top_line(model, gnssBlock, 2, model + "/Switch1", 1); % velocity GNSS
connect_top_line(model, kfBlock,   2, model + "/Switch1", 3); % velocity Kalman
connect_top_line(model, gnssBlock, 1, model + "/Switch2", 1); % position GNSS
connect_top_line(model, kfBlock,   1, model + "/Switch2", 3); % position Kalman

% Decouple selector lambda from the UKF correction enable.
kfSystem = char(kfBlock);
ukfBlock = kfSystem + "/Unscented Kalman Filter_";

if ~isempty(find_system(kfSystem, "SearchDepth", 1, "Name", "Fcn"))
    safe_delete_line(kfSystem, "Fcn", 1, "Unscented Kalman Filter_", 2);
    safe_delete_line(kfSystem, "lamda", 1, "Fcn", 1);
    delete_block(kfSystem + "/Fcn");
end

constBlock = kfSystem + "/Kalman Correction Enable";
if isempty(find_system(kfSystem, "SearchDepth", 1, "Name", "Kalman Correction Enable"))
    add_block("simulink/Sources/Constant", constBlock, ...
        "Value", "1", ...
        "Position", [415 110 445 140]);
else
    set_param(constBlock, "Value", "1");
end

termBlock = kfSystem + "/Term_lambda_unused";
if isempty(find_system(kfSystem, "SearchDepth", 1, "Name", "Term_lambda_unused"))
    add_block("simulink/Sinks/Terminator", termBlock, ...
        "Position", [420 170 445 195]);
end

connect_local_line(kfSystem, "Kalman Correction Enable", 1, "Unscented Kalman Filter_", 2);
connect_local_line(kfSystem, "lamda", 1, "Term_lambda_unused", 1);

% Keep the embedded FSM consistent with the external helper function.
rt = sfroot;
charts = rt.find("-isa", "Stateflow.EMChart");
for idx = 1:numel(charts)
    if contains(charts(idx).Path, model + "/INSTRUMENT DECISION1/Instrument Decision FSM")
        script = charts(idx).Script;
        script = strrep(script, "T_gnss_on = 30*3;   % [s] GNSS ON period before switching to Kalman", ...
            "T_gnss_on = 90;   % [s] GNSS ON period before switching to Kalman");
        script = strrep(script, "max_cov   = 2000;   % J threshold above which uncertainty is too large", ...
            "max_cov   = 2000; % J threshold above which uncertainty is too large");
        script = strrep(script, "timer_count = timer_count + Ts;", ...
            "timer_count = timer_count + int32(Ts);");
        script = strrep(script, "elseif (timer_count >= int32(T_gnss_on / Ts)) && (J < max_cov)", ...
            "elseif (timer_count >= int32(T_gnss_on)) && (J < max_cov)");
        charts(idx).Script = script;
    end
end

save_system(model);
fprintf("Applied Kalman/GNSS architecture fix to %s.\n", model);

end

function connect_top_line(systemName, srcBlock, srcPort, dstBlock, dstPort)
srcName = get_param(char(srcBlock), "Name");
dstName = get_param(char(dstBlock), "Name");
connect_local_line(char(systemName), srcName, srcPort, dstName, dstPort);
end

function connect_local_line(systemName, srcName, srcPort, dstName, dstPort)
delete_dst_line(systemName, dstName, dstPort);
add_line(systemName, sprintf("%s/%d", srcName, srcPort), ...
    sprintf("%s/%d", dstName, dstPort), "autorouting", "on");
end

function delete_dst_line(systemName, dstName, dstPort)
dstPath = string(systemName) + "/" + string(dstName);
if isempty(find_system(systemName, "SearchDepth", 1, "Name", dstName))
    return;
end
ph = get_param(char(dstPath), "PortHandles");
if dstPort <= numel(ph.Inport)
    lineHandle = get_param(ph.Inport(dstPort), "Line");
    if lineHandle ~= -1
        delete_line(lineHandle);
    end
end
end

function safe_delete_line(systemName, srcName, srcPort, dstName, dstPort)
if isempty(find_system(systemName, "SearchDepth", 1, "Name", srcName)) || ...
        isempty(find_system(systemName, "SearchDepth", 1, "Name", dstName))
    return;
end
try
    delete_line(systemName, sprintf("%s/%d", srcName, srcPort), ...
        sprintf("%s/%d", dstName, dstPort));
catch
    delete_dst_line(systemName, dstName, dstPort);
end
end

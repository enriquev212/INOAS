function report = diagnose_kalman_gnss_selection(stopTime)
%DIAGNOSE_KALMAN_GNSS_SELECTION Run and compare GNSS/Kalman/selected branches.

if nargin < 1
    stopTime = 600;
end

modelName = "MPCcontrolledSpacecraft_plant_gnss_kalman_decision";
codexStopTime = stopTime; %#ok<NASGU>

bdclose all;
load_system(modelName);

add_workspace_logger(modelName, "Spacecraft Dynamics", 1, "diag_plant_pos", [1160 80 1260 110]);
add_workspace_logger(modelName, "Spacecraft Dynamics", 2, "diag_plant_vel", [1160 120 1260 150]);
add_workspace_logger(modelName, "Enabled" + newline + "Subsystem1", 1, "diag_gnss_pos", [1160 170 1260 200]);
add_workspace_logger(modelName, "Enabled" + newline + "Subsystem1", 2, "diag_gnss_vel", [1160 210 1260 240]);
add_workspace_logger(modelName, "KALMAN FILTER", 1, "diag_kalman_pos", [1160 260 1260 290]);
add_workspace_logger(modelName, "KALMAN FILTER", 2, "diag_kalman_vel", [1160 300 1260 330]);
add_workspace_logger(modelName, "KALMAN FILTER", 3, "diag_kalman_cov", [1160 340 1260 370]);
add_workspace_logger(modelName, "Switch2", 1, "diag_selected_pos", [1160 390 1260 420]);
add_workspace_logger(modelName, "Switch1", 1, "diag_selected_vel", [1160 430 1260 460]);
add_workspace_logger(modelName, "MPC", 1, "diag_mpc_u", [1160 475 1260 505]);
add_workspace_logger(modelName, "INSTRUMENT DECISION1", 1, "diag_lambda_raw", [1160 520 1260 550]);

set_param(modelName, "StopTime", num2str(stopTime));

assignin("base", "modelName", modelName);
assignin("base", "codexStopTime", stopTime);
evalin("base", "Script_Concurso_Spacecraft_batch;");
t_ref = evalin("base", "t_ref");
x_ref_hist = evalin("base", "x_ref_hist");
h = evalin("base", "h");
simOut = sim(modelName, "StopTime", num2str(stopTime));

plantPos = get_logged_matrix(simOut, "diag_plant_pos");
gnssPos = get_logged_matrix(simOut, "diag_gnss_pos");
kalmanPos = get_logged_matrix(simOut, "diag_kalman_pos");
selectedPos = get_logged_matrix(simOut, "diag_selected_pos");
mpcU = get_logged_matrix(simOut, "diag_mpc_u");
lambda = get_logged_matrix(simOut, "diag_lambda_raw");

t = selectedPos.Time(:);
if exist("h", "var") && ~isempty(h)
    gridMask = abs(t / h - round(t / h)) < 1e-8;
    t = t(gridMask);
else
    gridMask = true(size(t));
end
refPos = interp1(t_ref(:), x_ref_hist(1:3, :).', t, "linear", "extrap");

plantData = interp_position(plantPos, t);
gnssData = interp_position(gnssPos, t);
kalmanData = interp_position(kalmanPos, t);
selectedData = interp_position(selectedPos, t);
mpcUData = interp_vector(mpcU, t);

errPlant = vecnorm(plantData - refPos, 2, 2);
errGnss = vecnorm(gnssData - refPos, 2, 2);
errKalman = vecnorm(kalmanData - refPos, 2, 2);
errSelected = vecnorm(selectedData - refPos, 2, 2);
errGnssVsPlant = vecnorm(gnssData - plantData, 2, 2);
errKalmanVsPlant = vecnorm(kalmanData - plantData, 2, 2);
errSelectedVsPlant = vecnorm(selectedData - plantData, 2, 2);

lambdaInterp = interp1(lambda.Time(:), lambda.Data(:), t, "previous", "extrap");

figDir = fullfile(pwd, "LaTeX", "GNSS", "figures", "gnss_sensor_validation");
if ~exist(figDir, "dir")
    mkdir(figDir);
end

figPath = fullfile(figDir, sprintf("kalman_gnss_selection_diag_%ds.png", round(stopTime)));
trackingFigPath = fullfile(figDir, sprintf("kalman_gnss_tracking_vs_reference_%ds.png", round(stopTime)));
navigationFigPath = fullfile(figDir, sprintf("kalman_gnss_navigation_vs_plant_%ds.png", round(stopTime)));
lambdaFigPath = fullfile(figDir, sprintf("kalman_gnss_lambda_%ds.png", round(stopTime)));
componentsFigPath = fullfile(figDir, sprintf("kalman_gnss_navigation_components_%ds.png", round(stopTime)));
matPath = fullfile(figDir, sprintf("kalman_gnss_selection_diag_%ds.mat", round(stopTime)));

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1300 720]);
tiledlayout(2,1, "TileSpacing", "compact");

nexttile;
hold on;
grid on;
plot(t, errPlant, "Color", [0.45 0.45 0.45], "LineWidth", 1.0, "DisplayName", "planta-ref");
plot(t, errGnss, "Color", [0.1 0.5 0.95], "LineWidth", 1.2, "DisplayName", "GNSS-ref");
plot(t, errKalman, "Color", [0.85 0.25 0.1], "LineWidth", 1.2, "DisplayName", "Kalman-ref");
plot(t, errSelected, "k", "LineWidth", 1.8, "DisplayName", "seleccionada-ref");
yline(20, "--", "20 m", "Color", [0.1 0.6 0.15], "LineWidth", 1.2, "DisplayName", "20 m");
yline(50, "--", "50 m", "Color", [0.95 0.45 0.05], "LineWidth", 1.2, "DisplayName", "50 m");
xlabel("Tiempo [s]");
ylabel("Error posicion [m]");
title("Diagnostico de ramas GNSS/Kalman/seleccion");
legend("Location", "best");

nexttile;
stairs(t, lambdaInterp, "LineWidth", 1.4, "DisplayName", "lambda selector");
grid on;
ylim([-0.1 1.1]);
xlabel("Tiempo [s]");
ylabel("lambda");
title("lambda = 1 GNSS, lambda = 0 Kalman");

exportgraphics(fig, figPath, "Resolution", 180);
close(fig);

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1300 620]);
hold on;
grid on;
plot(t, errPlant, "Color", [0.45 0.45 0.45], "LineWidth", 1.1, "DisplayName", "planta-ref");
plot(t, errGnss, "Color", [0.1 0.5 0.95], "LineWidth", 1.2, "DisplayName", "GNSS-ref");
plot(t, errKalman, "Color", [0.85 0.25 0.1], "LineWidth", 1.2, "DisplayName", "Kalman-ref");
plot(t, errSelected, "k", "LineWidth", 1.8, "DisplayName", "seleccionada-ref");
yline(20, "--", "20 m", "Color", [0.1 0.6 0.15], "LineWidth", 1.2, "DisplayName", "20 m");
yline(50, "--", "50 m", "Color", [0.95 0.45 0.05], "LineWidth", 1.2, "DisplayName", "50 m");
xlabel("Tiempo [s]");
ylabel("Norma del error de seguimiento [m]");
title("Seguimiento respecto a la trayectoria de referencia");
legend("Location", "best");
exportgraphics(fig, trackingFigPath, "Resolution", 180);
close(fig);

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1300 620]);
hold on;
grid on;
plot(t, errGnssVsPlant, "Color", [0.1 0.5 0.95], "LineWidth", 1.2, "DisplayName", "GNSS-planta");
plot(t, errKalmanVsPlant, "Color", [0.85 0.25 0.1], "LineWidth", 1.2, "DisplayName", "Kalman-planta");
plot(t, errSelectedVsPlant, "k", "LineWidth", 1.8, "DisplayName", "seleccionada-planta");
yline(20, "--", "20 m", "Color", [0.1 0.6 0.15], "LineWidth", 1.2, "DisplayName", "20 m");
yline(50, "--", "50 m", "Color", [0.95 0.45 0.05], "LineWidth", 1.2, "DisplayName", "50 m");
xlabel("Tiempo [s]");
ylabel("Norma del error de navegacion [m]");
title("Error real de navegacion respecto a la planta");
legend("Location", "best");
exportgraphics(fig, navigationFigPath, "Resolution", 180);
close(fig);

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1300 420]);
stairs(t, lambdaInterp, "LineWidth", 1.8, "Color", [0 0.35 0.7]);
grid on;
ylim([-0.1 1.1]);
xlabel("Tiempo [s]");
ylabel("lambda");
title("Selector instrumental: lambda = 1 GNSS, lambda = 0 Kalman");
exportgraphics(fig, lambdaFigPath, "Resolution", 180);
close(fig);

selectedMinusPlant = selectedData - plantData;
kalmanMinusPlant = kalmanData - plantData;
gnssMinusPlant = gnssData - plantData;

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1300 820]);
tiledlayout(3,1, "TileSpacing", "compact");
axisNames = ["X", "Y", "Z"];
for axisIdx = 1:3
    nexttile;
    hold on;
    grid on;
    plot(t, gnssMinusPlant(:, axisIdx), "Color", [0.1 0.5 0.95], "LineWidth", 1.0, "DisplayName", "GNSS-planta");
    plot(t, kalmanMinusPlant(:, axisIdx), "Color", [0.85 0.25 0.1], "LineWidth", 1.0, "DisplayName", "Kalman-planta");
    plot(t, selectedMinusPlant(:, axisIdx), "k", "LineWidth", 1.4, "DisplayName", "seleccionada-planta");
    ylabel(axisNames(axisIdx) + " [m]");
    if axisIdx == 1
        title("Componentes del error de navegacion");
        legend("Location", "best");
    end
    if axisIdx == 3
        xlabel("Tiempo [s]");
    end
end
exportgraphics(fig, componentsFigPath, "Resolution", 180);
close(fig);

report = struct();
report.stopTime = stopTime;
report.t = t;
report.errPlant = errPlant;
report.errGnss = errGnss;
report.errKalman = errKalman;
report.errSelected = errSelected;
report.errGnssVsPlant = errGnssVsPlant;
report.errKalmanVsPlant = errKalmanVsPlant;
report.errSelectedVsPlant = errSelectedVsPlant;
report.refPos = refPos;
report.plantPos = plantData;
report.gnssPos = gnssData;
report.kalmanPos = kalmanData;
report.selectedPos = selectedData;
report.mpcU = mpcUData;
report.plantMinusRef = plantData - refPos;
report.gnssMinusRef = gnssData - refPos;
report.kalmanMinusRef = kalmanData - refPos;
report.selectedMinusRef = selectedData - refPos;
report.gnssMinusPlant = gnssData - plantData;
report.kalmanMinusPlant = kalmanData - plantData;
report.selectedMinusPlant = selectedData - plantData;
report.lambda = lambdaInterp;
if evalin("base", "exist('referenceSourceMode','var')")
    report.referenceSourceMode = string(evalin("base", "referenceSourceMode"));
end
if evalin("base", "exist('referenceMeta','var')")
    report.referenceMeta = evalin("base", "referenceMeta");
end
if evalin("base", "exist('t_debris','var')")
    report.tDebris = evalin("base", "t_debris");
end
if evalin("base", "exist('dsafe0','var')")
    report.dsafe0 = evalin("base", "dsafe0");
end
report.figPath = figPath;
report.trackingFigPath = trackingFigPath;
report.navigationFigPath = navigationFigPath;
report.lambdaFigPath = lambdaFigPath;
report.componentsFigPath = componentsFigPath;
report.matPath = matPath;
report.summary = table( ...
    {'plant'; 'gnss'; 'kalman'; 'selected'}, ...
    [errPlant(end); errGnss(end); errKalman(end); errSelected(end)], ...
    [max(errPlant); max(errGnss); max(errKalman); max(errSelected)], ...
    [rms(errPlant); rms(errGnss); rms(errKalman); rms(errSelected)], ...
    'VariableNames', {'branch', 'final_m', 'max_m', 'rms_m'});

report.navigationError = table( ...
    {'gnss_minus_plant'; 'kalman_minus_plant'; 'selected_minus_plant'}, ...
    [errGnssVsPlant(end); errKalmanVsPlant(end); errSelectedVsPlant(end)], ...
    [max(errGnssVsPlant); max(errKalmanVsPlant); max(errSelectedVsPlant)], ...
    [rms(errGnssVsPlant); rms(errKalmanVsPlant); rms(errSelectedVsPlant)], ...
    'VariableNames', {'branch', 'final_m', 'max_m', 'rms_m'});

save(matPath, "report");
disp(report.summary);
disp(report.navigationError);
fprintf("Saved diagnostic figure: %s\n", figPath);
fprintf("Saved tracking figure:   %s\n", trackingFigPath);
fprintf("Saved navigation figure: %s\n", navigationFigPath);
fprintf("Saved lambda figure:     %s\n", lambdaFigPath);
fprintf("Saved components figure: %s\n", componentsFigPath);
fprintf("Saved diagnostic data:   %s\n", matPath);

close_system(modelName, 0);

end

function add_workspace_logger(modelName, blockName, outPort, signalName, position)
blockPath = string(modelName) + "/" + string(blockName);
if ~bdIsLoaded(modelName)
    load_system(modelName);
end
if isempty(find_system(modelName, "SearchDepth", 1, "Name", blockName))
    warning("Block not found for diagnostic logging: %s", blockPath);
    return;
end
ph = get_param(char(blockPath), "PortHandles");
if outPort > numel(ph.Outport)
    warning("Output port %d not found on block %s", outPort, blockPath);
    return;
end
lineHandle = get_param(ph.Outport(outPort), "Line");
if lineHandle == -1
    warning("No line on %s/%d", blockPath, outPort);
    return;
end

loggerName = "diag_ws_" + string(signalName);
loggerPath = string(modelName) + "/" + loggerName;
old = find_system(modelName, "SearchDepth", 1, "Name", loggerName);
if ~isempty(old)
    delete_block(char(loggerPath));
end

add_block("simulink/Sinks/To Workspace", char(loggerPath), ...
    "VariableName", char(signalName), ...
    "SaveFormat", "Timeseries", ...
    "MaxDataPoints", "inf", ...
    "Position", position);

add_line(char(modelName), sprintf("%s/%d", blockName, outPort), ...
    sprintf("%s/1", loggerName), "autorouting", "on");
end

function out = get_logged_matrix(simOut, signalName)
ts = simOut.get(signalName);
data = squeeze(ts.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) < size(data, 2) && size(data, 1) <= 6
    data = data.';
end
out = struct("Time", ts.Time(:), "Data", data);
end

function pos = interp_position(signal, t)
data = signal.Data;
if size(data, 2) > 3
    data = data(:, 1:3);
end
pos = interp1(signal.Time(:), data, t, "linear", "extrap");
end

function vec = interp_vector(signal, t)
data = signal.Data;
if isvector(data)
    data = data(:);
end
vec = interp1(signal.Time(:), data, t, "previous", "extrap");
end

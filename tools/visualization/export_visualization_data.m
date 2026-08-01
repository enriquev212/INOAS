function outputFile = export_visualization_data(outputFile)
%EXPORT_VISUALIZATION_DATA Export compact INOAS results for Python visuals.
%
% Run this after a Simulink simulation has produced the variable "out":
%
%   out = sim("inoas_model");
%   export_visualization_data
%
% The generated MAT file is intentionally compact and contains only the
% signals needed by the Python visualization script.

if nargin < 1 || isempty(outputFile)
    repoRoot = inoasProjectRoot();
    outputDir = fullfile(repoRoot, "results", "visualization");
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end
    outputFile = fullfile(outputDir, "inoas_visualization_data.mat");
end

if ~evalin("base", "exist('out', 'var')")
    error("export_visualization_data:MissingSimulationOutput", ...
        "Run a simulation first so the base workspace contains the variable 'out'.");
end

simOut = evalin("base", "out");
logsout = simOut.logsout;

truthSignal = getRequiredLogSignal(logsout, "X_perfect_sensor");
estimatedSignal = getRequiredLogSignal(logsout, "Estimated_Pos_x");
controlSignal = getRequiredLogSignal(logsout, ["u_MPC", "u_discret"]);

[time_s, truth_eci_m] = signalToMatrix(truthSignal, 3, "X_perfect_sensor");
[estimate_time_s, estimated_eci_m] = signalToMatrix(estimatedSignal, 3, "Estimated_Pos_x");
[control_time_s, control_eci_mps2] = signalToMatrix(controlSignal, 3, "u_MPC");

time_ref = getBase("time_ref", []);
if isempty(time_ref)
    error("export_visualization_data:MissingReferenceTime", ...
        "The base workspace variable 'time_ref' is required.");
end
time_ref = time_ref(:);

reference_eci_m = [
    getTimeseriesData("ref_ts_x"), ...
    getTimeseriesData("ref_ts_y"), ...
    getTimeseriesData("ref_ts_z")];

reference_velocity_eci_mps = [
    getTimeseriesData("ref_ts_vx"), ...
    getTimeseriesData("ref_ts_vy"), ...
    getTimeseriesData("ref_ts_vz")];

reference_eci_m = interp1(time_ref, reference_eci_m, time_s, "pchip", "extrap");
reference_velocity_eci_mps = interp1(time_ref, reference_velocity_eci_mps, time_s, "pchip", "extrap");

[debris_time_s, debris_eci_m, debris_eci_m_at_time] = debrisTrajectory(time_s);

t_debris_s = scalarBase("t_debris", NaN);
safe_radius_m = scalarBase("dsafe0", NaN);
u_max_mps2 = scalarBase("u_max", NaN);
mpc_horizon = scalarBase("Np", NaN);
sample_time_s = scalarBase("h", NaN);

[dynamic_safe_time_s, dynamic_safe_first_m, dynamic_safe_horizon_m] = dynamicSafetyLog();

[lambda_time_s, lambda] = optionalSignal(logsout, ...
    ["lambda", "instrument_lambda", "gnss_lambda", "GNSS_selector", ...
     "InstrumentDecision", "instrument_decision", "lambda_decision"]);

[gnss_quality_time_s, gnss_nsv, gnss_pdop, gnss_hpe_m, gnss_vpe_m, gnss_solution_flag] = gnssQualitySignals();

metadata = struct();
metadata.created_by = "export_visualization_data";
metadata.description = "Compact INOAS simulation export for Python visualizations";
metadata.model = "inoas_model";

save(outputFile, ...
    "time_s", "truth_eci_m", "estimate_time_s", "estimated_eci_m", ...
    "reference_eci_m", "reference_velocity_eci_mps", ...
    "control_time_s", "control_eci_mps2", ...
    "debris_time_s", "debris_eci_m", "debris_eci_m_at_time", ...
    "t_debris_s", "safe_radius_m", "u_max_mps2", "mpc_horizon", "sample_time_s", ...
    "dynamic_safe_time_s", "dynamic_safe_first_m", "dynamic_safe_horizon_m", ...
    "lambda_time_s", "lambda", ...
    "gnss_quality_time_s", "gnss_nsv", "gnss_pdop", "gnss_hpe_m", "gnss_vpe_m", ...
    "gnss_solution_flag", "metadata", "-v7");

fprintf("\nINOAS visualization data exported:\n%s\n", outputFile);

end

function repoRoot = inoasProjectRoot()
thisFile = mfilename("fullpath");
repoRoot = fileparts(fileparts(fileparts(thisFile)));
end

function value = getBase(name, defaultValue)
if evalin("base", "exist('" + name + "', 'var')")
    value = evalin("base", name);
else
    value = defaultValue;
end
end

function value = scalarBase(name, defaultValue)
value = getBase(name, defaultValue);
if isempty(value)
    value = defaultValue;
else
    value = double(value(1));
end
end

function data = getTimeseriesData(name)
ts = getBase(name, []);
if isempty(ts)
    error("export_visualization_data:MissingTimeseries", ...
        "The base workspace timeseries '%s' is required.", name);
end
data = ts.Data(:);
end

function signal = getRequiredLogSignal(logsout, candidateNames)
signal = getOptionalLogSignal(logsout, candidateNames);
if isempty(signal)
    error("export_visualization_data:MissingLogSignal", ...
        "None of these logged signals were found: %s", strjoin(candidateNames, ", "));
end
end

function signal = getOptionalLogSignal(logsout, candidateNames)
signal = [];
availableNames = string(logsout.getElementNames);
for k = 1:numel(candidateNames)
    name = string(candidateNames(k));
    if any(availableNames == name)
        signal = logsout.get(char(name)).Values;
        return
    end
end
end

function [time, data] = signalToMatrix(signal, columns, signalName)
time = signal.Time(:);
data = squeeze(signal.Data);

if isvector(data)
    data = data(:);
end

if size(data, 2) == columns
    return
end

if size(data, 1) == columns
    data = data.';
    return
end

error("export_visualization_data:UnexpectedSignalShape", ...
    "Signal '%s' must have %d columns after squeezing; got %s.", ...
    signalName, columns, mat2str(size(data)));
end

function [time, values] = optionalSignal(logsout, candidateNames)
signal = getOptionalLogSignal(logsout, candidateNames);
if isempty(signal)
    time = [];
    values = [];
    return
end

time = signal.Time(:);
values = squeeze(signal.Data);
values = values(:);
end

function [debrisTime, debrisTrajectoryData, debrisAtTime] = debrisTrajectory(time)
xDebrisHist = getBase("x_debris_hist", []);

if isempty(xDebrisHist)
    rkDebris = getBase("rk_debris", []);
    if isempty(rkDebris)
        debrisTime = [];
        debrisTrajectoryData = [];
        debrisAtTime = [];
    else
        debrisTime = time(:);
        debrisTrajectoryData = repmat(rkDebris(:).', numel(time), 1);
        debrisAtTime = debrisTrajectoryData;
    end
    return
end

if size(xDebrisHist, 1) ~= 6 && size(xDebrisHist, 2) == 6
    xDebrisHist = xDebrisHist.';
end

debrisTime = getBase("t_debris_ref", []);
if isempty(debrisTime)
    timeRef = getBase("time_ref", []);
    debrisTime = timeRef(1:size(xDebrisHist, 2));
end
debrisTime = debrisTime(:);

debrisTrajectoryData = xDebrisHist(1:3, :).';
debrisAtTime = interp1(debrisTime, debrisTrajectoryData, time, "pchip", "extrap");
end

function [time, firstRadius, horizonRadius] = dynamicSafetyLog()
time = getBase("mpc_dsafe_log_time", []);
firstRadius = getBase("mpc_dsafe_log_first", []);
horizonRadius = getBase("mpc_dsafe_log_max", []);

time = time(:);
firstRadius = firstRadius(:);
horizonRadius = horizonRadius(:);
end

function [time, nsv, pdop, hpe, vpe, solutionFlag] = gnssQualitySignals()
[time, nsv] = timeseriesOrEmpty("ts_gnss_nsv");
[pdopTime, pdop] = timeseriesOrEmpty("ts_gnss_pdop");
[hpeTime, hpe] = timeseriesOrEmpty("ts_gnss_hpe");
[vpeTime, vpe] = timeseriesOrEmpty("ts_gnss_vpe");
[solutionTime, solutionFlag] = timeseriesOrEmpty("ts_gnss_sol");

if isempty(time)
    time = pdopTime;
end

if isempty(time)
    time = hpeTime;
end

if isempty(time)
    time = vpeTime;
end

if isempty(time)
    time = solutionTime;
end

if ~isempty(time)
    pdop = resampleOptional(pdopTime, pdop, time);
    hpe = resampleOptional(hpeTime, hpe, time);
    vpe = resampleOptional(vpeTime, vpe, time);
    solutionFlag = resampleOptional(solutionTime, solutionFlag, time);
end
end

function [time, data] = timeseriesOrEmpty(name)
ts = getBase(name, []);
if isempty(ts)
    time = [];
    data = [];
else
    time = ts.Time(:);
    data = squeeze(ts.Data);
    data = data(:);
end
end

function dataOut = resampleOptional(timeIn, dataIn, timeOut)
if isempty(timeIn) || isempty(dataIn)
    dataOut = [];
else
    dataOut = interp1(timeIn, dataIn, timeOut, "previous", "extrap");
end
end

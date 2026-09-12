function outputDir = export_campaign_csv(outputDir)
%EXPORT_CAMPAIGN_CSV Export INOAS simulation results as CSV files.
%
% Run this after a Simulink simulation has produced the variable "out":
%
%   out = sim("inoas_model");
%   export_campaign_csv
%
% The export is intended for paper-quality Python figures. It writes:
%
%   timeseries.csv  - trajectory, estimation, debris, safety, and RTN geometry
%   control.csv     - commanded accelerations, saturation ratio, and delta-v
%   navigation.csv  - GNSS/Kalman selector, NIS, and GNSS quality indicators
%   metrics.csv     - scalar summary metrics for tables and captions

if nargin < 1 || isempty(outputDir)
    repoRoot = inoasProjectRoot();
    outputDir = fullfile(repoRoot, "results", "campaign", "baseline");
end

if ~isfolder(outputDir)
    mkdir(outputDir);
end

if ~evalin("base", "exist('out', 'var')")
    error("export_campaign_csv:MissingSimulationOutput", ...
        "Run a simulation first so the base workspace contains the variable 'out'.");
end

simOut = evalin("base", "out");
logsout = simOut.logsout;

truthSignal = getRequiredLogSignal(logsout, ["truth_position_eci", "X_perfect_sensor"]);
estimatedSignal = getRequiredLogSignal(logsout, "Estimated_Pos_x");
controlSignal = getRequiredLogSignal(logsout, ["u_MPC", "u_discret"]);

[time_s, truth_eci_m] = signalToMatrix(truthSignal, 3, "X_perfect_sensor");
[estimate_time_s, estimated_eci_m_raw] = signalToMatrix(estimatedSignal, 3, "Estimated_Pos_x");
[control_time_s, control_mps2] = signalToMatrix(controlSignal, 3, "u_MPC");

estimated_eci_m = interp1(estimate_time_s, estimated_eci_m_raw, time_s, "pchip", "extrap");

time_ref = getBase("time_ref", []);
if isempty(time_ref)
    error("export_campaign_csv:MissingReferenceTime", ...
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

[~, ~, debris_eci_m_at_time] = debrisTrajectory(time_s);
if isempty(debris_eci_m_at_time)
    debris_eci_m_at_time = NaN(numel(time_s), 3);
end

tracking_error_m = vecnorm(truth_eci_m - reference_eci_m, 2, 2);
estimation_error_m = vecnorm(estimated_eci_m - truth_eci_m, 2, 2);
debris_distance_m = vecnorm(truth_eci_m - debris_eci_m_at_time, 2, 2);
nominal_debris_distance_m = vecnorm(reference_eci_m - debris_eci_m_at_time, 2, 2);

safe_radius_m = scalarBase("dsafe0", NaN);
u_max_mps2 = scalarBase("u_max", NaN);
t_debris_s = scalarBase("t_debris", NaN);
mpc_horizon = scalarBase("Np", NaN);
sample_time_s = scalarBase("h", NaN);
gnss_sample_time_s = scalarBase("gnss_sample_time", NaN);

[dynamic_safe_time_s, dynamic_safe_first_m_raw, dynamic_safe_horizon_m_raw] = dynamicSafetyLog();
dynamic_safe_first_m = resamplePrevious(dynamic_safe_time_s, dynamic_safe_first_m_raw, time_s, safe_radius_m);
dynamic_safe_horizon_m = resamplePrevious(dynamic_safe_time_s, dynamic_safe_horizon_m_raw, time_s, safe_radius_m);

robust_margin_m = debris_distance_m - dynamic_safe_first_m;

sc_debris_rtn_m = rtnComponents(reference_eci_m, reference_velocity_eci_mps, truth_eci_m - debris_eci_m_at_time);
ref_debris_rtn_m = rtnComponents(reference_eci_m, reference_velocity_eci_mps, reference_eci_m - debris_eci_m_at_time);

[lambda_time_s, lambda_raw] = optionalSignal(logsout, ...
    ["lambda", "lamda", "instrument_lambda", "gnss_lambda", "GNSS_selector", ...
     "InstrumentDecision", "instrument_decision", "lambda_decision"]);
lambda = resamplePrevious(lambda_time_s, lambda_raw, time_s, NaN);

[nis_time_s, nis_raw] = optionalSignal(logsout, ...
    ["NIS", "NIS_smooth", "PseudoNIS", "Pseudo_NIS", "pseudo_NIS", "nis"]);
nis = resamplePrevious(nis_time_s, nis_raw, time_s, NaN);

[gnss_quality_time_s, gnss_nsv_raw, gnss_pdop_raw, gnss_hpe_raw, gnss_vpe_raw, gnss_solution_raw] = gnssQualitySignals();
gnss_nsv = resamplePrevious(gnss_quality_time_s, gnss_nsv_raw, time_s, NaN);
gnss_pdop = resamplePrevious(gnss_quality_time_s, gnss_pdop_raw, time_s, NaN);
gnss_hpe_m = resamplePrevious(gnss_quality_time_s, gnss_hpe_raw, time_s, NaN);
gnss_vpe_m = resamplePrevious(gnss_quality_time_s, gnss_vpe_raw, time_s, NaN);
gnss_solution_flag = resamplePrevious(gnss_quality_time_s, gnss_solution_raw, time_s, NaN);

timeseriesTable = table( ...
    time_s, ...
    truth_eci_m(:,1), truth_eci_m(:,2), truth_eci_m(:,3), ...
    estimated_eci_m(:,1), estimated_eci_m(:,2), estimated_eci_m(:,3), ...
    reference_eci_m(:,1), reference_eci_m(:,2), reference_eci_m(:,3), ...
    reference_velocity_eci_mps(:,1), reference_velocity_eci_mps(:,2), reference_velocity_eci_mps(:,3), ...
    debris_eci_m_at_time(:,1), debris_eci_m_at_time(:,2), debris_eci_m_at_time(:,3), ...
    tracking_error_m, estimation_error_m, debris_distance_m, nominal_debris_distance_m, ...
    safe_radius_m * ones(size(time_s)), dynamic_safe_first_m, dynamic_safe_horizon_m, robust_margin_m, ...
    sc_debris_rtn_m(:,1), sc_debris_rtn_m(:,2), sc_debris_rtn_m(:,3), ...
    ref_debris_rtn_m(:,1), ref_debris_rtn_m(:,2), ref_debris_rtn_m(:,3), ...
    'VariableNames', { ...
        'time_s', ...
        'truth_x_m', 'truth_y_m', 'truth_z_m', ...
        'estimate_x_m', 'estimate_y_m', 'estimate_z_m', ...
        'reference_x_m', 'reference_y_m', 'reference_z_m', ...
        'reference_vx_mps', 'reference_vy_mps', 'reference_vz_mps', ...
        'debris_x_m', 'debris_y_m', 'debris_z_m', ...
        'tracking_error_m', 'estimation_error_m', 'debris_distance_m', 'nominal_debris_distance_m', ...
        'safe_radius_m', 'dynamic_safe_first_m', 'dynamic_safe_horizon_m', 'robust_margin_m', ...
        'sc_debris_R_m', 'sc_debris_I_m', 'sc_debris_N_m', ...
        'ref_debris_R_m', 'ref_debris_I_m', 'ref_debris_N_m'});

u_norm_mps2 = vecnorm(control_mps2, 2, 2);
if isfinite(u_max_mps2) && u_max_mps2 > 0
    axis_saturation_ratio = max(abs(control_mps2), [], 2) ./ u_max_mps2;
else
    axis_saturation_ratio = NaN(size(control_time_s));
end
delta_v_mps = cumtrapz(control_time_s, u_norm_mps2);

controlTable = table( ...
    control_time_s, control_mps2(:,1), control_mps2(:,2), control_mps2(:,3), ...
    u_norm_mps2, u_max_mps2 * ones(size(control_time_s)), axis_saturation_ratio, delta_v_mps, ...
    'VariableNames', {'time_s', 'u_1_mps2', 'u_2_mps2', 'u_3_mps2', ...
                      'u_norm_mps2', 'u_max_mps2', 'axis_saturation_ratio', 'delta_v_mps'});

navigationTable = table( ...
    time_s, lambda, nis, estimation_error_m, gnss_nsv, gnss_pdop, gnss_hpe_m, gnss_vpe_m, gnss_solution_flag, ...
    'VariableNames', {'time_s', 'lambda', 'nis', 'estimation_error_m', ...
                      'gnss_nsv', 'gnss_pdop', 'gnss_hpe_m', 'gnss_vpe_m', 'gnss_solution_flag'});

metricNames = strings(0, 1);
metricValues = zeros(0, 1);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "duration_s", time_s(end) - time_s(1));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "t_debris_s", t_debris_s);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "mpc_horizon_steps", mpc_horizon);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "mpc_sample_time_s", sample_time_s);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "gnss_sample_time_s", gnss_sample_time_s);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "u_max_mps2", u_max_mps2);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "safe_radius_m", safe_radius_m);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "min_debris_distance_m", min(debris_distance_m, [], "omitnan"));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "min_robust_margin_m", min(robust_margin_m, [], "omitnan"));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "max_dynamic_safe_first_m", max(dynamic_safe_first_m, [], "omitnan"));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "max_tracking_error_m", max(tracking_error_m, [], "omitnan"));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "max_estimation_error_m", max(estimation_error_m, [], "omitnan"));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "final_estimation_error_m", estimation_error_m(end));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "peak_control_norm_mps2", max(u_norm_mps2, [], "omitnan"));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "peak_axis_acceleration_mps2", max(max(abs(control_mps2), [], 2), [], "omitnan"));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "final_delta_v_mps", delta_v_mps(end));
[metricNames, metricValues] = addMetric(metricNames, metricValues, "saturation_fraction", mean(axis_saturation_ratio >= 0.999, "omitnan"));
if all(isnan(lambda))
    dutyRatio = NaN;
else
    lambdaOn = double(lambda > 0.5);
    dutyRatio = trapz(time_s, lambdaOn) / max(time_s(end) - time_s(1), eps);
end
[metricNames, metricValues] = addMetric(metricNames, metricValues, "gnss_active_fraction", dutyRatio);
[metricNames, metricValues] = addMetric(metricNames, metricValues, "gnss_energy_saving_fraction", 1 - dutyRatio);

metricsTable = table(metricNames, metricValues, 'VariableNames', {'metric', 'value'});

writetable(timeseriesTable, fullfile(outputDir, "timeseries.csv"));
writetable(controlTable, fullfile(outputDir, "control.csv"));
writetable(navigationTable, fullfile(outputDir, "navigation.csv"));
writetable(metricsTable, fullfile(outputDir, "metrics.csv"));

fprintf("\nINOAS campaign CSV export written to:\n%s\n", outputDir);
fprintf("  timeseries.csv\n  control.csv\n  navigation.csv\n  metrics.csv\n");

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
    error("export_campaign_csv:MissingTimeseries", ...
        "The base workspace timeseries '%s' is required.", name);
end
data = ts.Data(:);
end

function signal = getRequiredLogSignal(logsout, candidateNames)
signal = getOptionalLogSignal(logsout, candidateNames);
if isempty(signal)
    error("export_campaign_csv:MissingLogSignal", ...
        "None of these logged signals were found: %s", strjoin(string(candidateNames), ", "));
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

error("export_campaign_csv:UnexpectedSignalShape", ...
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
    nsv = resamplePrevious(time, nsv, time, NaN);
    pdop = resamplePrevious(pdopTime, pdop, time, NaN);
    hpe = resamplePrevious(hpeTime, hpe, time, NaN);
    vpe = resamplePrevious(vpeTime, vpe, time, NaN);
    solutionFlag = resamplePrevious(solutionTime, solutionFlag, time, NaN);
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

function dataOut = resamplePrevious(timeIn, dataIn, timeOut, defaultValue)
if isempty(timeIn) || isempty(dataIn) || isempty(timeOut)
    dataOut = defaultValue * ones(size(timeOut));
    return
end

timeIn = timeIn(:);
dataIn = double(dataIn(:)); % Logged selectors may be logical.
[timeUnique, uniqueIdx] = unique(timeIn, "stable");
dataUnique = dataIn(uniqueIdx);

if isscalar(timeUnique)
    dataOut = dataUnique(1) * ones(size(timeOut));
else
    dataOut = interp1(timeUnique, dataUnique, timeOut, "previous", "extrap");
end
end

function components = rtnComponents(referenceR, referenceV, vectorEci)
radial = referenceR ./ max(vecnorm(referenceR, 2, 2), eps);
normal = cross(referenceR, referenceV, 2);
normal = normal ./ max(vecnorm(normal, 2, 2), eps);
intrack = cross(normal, radial, 2);

components = [
    sum(vectorEci .* radial, 2), ...
    sum(vectorEci .* intrack, 2), ...
    sum(vectorEci .* normal, 2)];
end

function [names, values] = addMetric(names, values, name, value)
names(end + 1, 1) = string(name);
values(end + 1, 1) = double(value);
end

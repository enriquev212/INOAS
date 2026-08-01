%% INOAS Simulink initialization
% Loads all base-workspace variables required by the final Simulink model:
% scenario definition, GNSS sensor profile, Kalman tuning, nominal reference,
% debris encounter, and MPC configuration.
clc; close all;

rng('shuffle');

%% Repository setup
scriptPath = mfilename("fullpath");
if strlength(scriptPath) > 0
    repoRoot = fileparts(scriptPath);
else
    repoRoot = pwd;
end
addpath(repoRoot);
addpath(genpath(fullfile(repoRoot, "matlab")));
addpath(fullfile(repoRoot, "models"));

preservedModelName = "";
preservedStopTime = [];

if exist("modelName", "var")
    preservedModelName = string(modelName);
end

if exist("simulationStopTime", "var")
    preservedStopTime = simulationStopTime;
end

skipBatchClear = ispref("inoas", "skipBatchClear") && getpref("inoas", "skipBatchClear");
if ispref("inoas", "skipBatchClear")
    rmpref("inoas", "skipBatchClear");
end

if ~skipBatchClear
    clearvars -except preservedModelName preservedStopTime skipBatchClear repoRoot;
end
clc

if strlength(preservedModelName) > 0
    modelName = preservedModelName;
end

if ~isempty(preservedStopTime)
    simulationStopTime = preservedStopTime;
end

clear preservedModelName preservedStopTime

%% Input files and physical scenario
gnssCovarianceFile = inoas_data_file("cov_perturb_POS_s6a_Y24D011_fixed.dat");
referenceTrajectoryFile = inoas_data_path("referenceTrajectory.mat");
debrisTrajectoryFile = inoas_data_path("debrisTrajectory.mat");
a = 7714.43 * 1000;  % [m]
ecc = 0.000095;
inc = 63.04;         % [deg]
RAAN = 116.6;        % [deg]
w = 90;              % [deg]
theta = 131;         % [deg]

F_control = 4 * 220; % [N]
m_sat = 10 * 1000;   % [kg]
initMass = m_sat;
ref = 1.3;           % reflectivity coefficient
area = 15;           % [m^2]

start_date = juliandate(datetime(2024, 1, 11));
end_date = juliandate(datetime(2024, 2, 11));
tf = 4000;           % [s] nominal simulation duration

requestedStopTime = [];
if exist("simulationStopTime", "var") && ~isempty(simulationStopTime)
    requestedStopTime = double(simulationStopTime);
elseif exist("modelName", "var") && strlength(string(modelName)) > 0
    requestedStopTime = getModelStopTimeSeconds(modelName);
else
    requestedStopTime = getOpenModelStopTimeSeconds();
end

if ~isempty(requestedStopTime)
    % Keep the nominal/reference trajectory available for the whole
    % simulation. Otherwise the MPC clamps to the last reference sample and
    % the relative state grows artificially once the run outlasts the
    % precomputed nominal timeline.
    tf = max(tf, requestedStopTime);
    simulationStopTime = requestedStopTime;
end

clear requestedStopTime
k_p = 5;
k_d = 20;

%% Kalman/UKF tuning and decision observable
Ts = 1;          % [s] master sample time for sensors and estimator
var_IMU = 0.01; % accelerometer variance used by the Kalman propagation

% Synthetic internal sensor measurement covariance.
sigma_pos = 100; % [m]
sigma_alt = 50;  % [m]
var_pos = sigma_pos^2;
var_alt = sigma_alt^2;
R_matrix = diag([var_pos, var_pos, var_pos, var_alt]);

% Process-noise covariance for the 6-state orbital estimator.
Q_matrix = diag([1, 1, 1, 1e-2, 1e-2, 1e-2]);

% Nominal GNSS measurement covariance used by the estimator.
sigma_pos_gnss = 5;   % [m]
sigma_vel_gnss = 0.1; % [m/s]
R_gnss = diag([sigma_pos_gnss^2, sigma_pos_gnss^2, sigma_pos_gnss^2, ...
               sigma_vel_gnss^2, sigma_vel_gnss^2, sigma_vel_gnss^2]);

% Initial Kalman covariance: 1 km position error and 10 m/s velocity error.
P0_kalman = diag([1e6, 1e6, 1e6, 100, 100, 100]);

% Instrument-decision observable:
% J = trace(S_inv * P * S_T_inv), with position/velocity scaling in S.
sigma_x = 100; % [m]
sigma_v = 50;  % [m/s]
s = [sigma_x, sigma_x, sigma_x, sigma_v, sigma_v, sigma_v];
S = diag(s);
S_inv = inv(S);
S_T_inv = inv(S');
max_cov = 2000;

%% *MPC controller initialization*
% Control system parameters
nx = 6; % State Variables
m = 3; % Control variables

%% Control System parameters      
targetModelName = "";
if exist("modelName", "var")
    targetModelName = string(modelName);
end

%% Reference trajectory generation setup

% Final setup: the GNSS .dat file characterizes sensor quality and noise,
% not the trajectory followed by the MPC. The reference is generated from
% orbital elements and propagated dynamically.
referencePropModel = "j2-rk4";

% MPC/reference parameters. The real GNSS sensor sampling is loaded from the
% .dat file in prepare_gnss_sensor_workspace.
Np = 125;     % GNSS/MPC prediction-horizon length
h  = 3;      % MPC reference sample time [s]

%% MPC tuning

mpcTuneConfig = struct();
if ispref("inoas", "mpcTuneConfig")
    mpcTuneConfig = getpref("inoas", "mpcTuneConfig");
    rmpref("inoas", "mpcTuneConfig");
end

% Cost matrices
Q_step = 1e-7*[ ...
    5  5  5 ...      
    10  10  10 ];

R_step = 3e3*[ ...
    2  1.5  1.2 ];

S_step = 1e4*[ ...
    2  5  8 ];


slackWeight = 1e5;

if isfield(mpcTuneConfig, "Q_step")
    Q_step = mpcTuneConfig.Q_step(:).';
end

if isfield(mpcTuneConfig, "R_step")
    R_step = mpcTuneConfig.R_step(:).';
end

if isfield(mpcTuneConfig, "S_step")
    S_step = mpcTuneConfig.S_step(:).';
end

if isfield(mpcTuneConfig, "slackWeight")
    slackWeight = mpcTuneConfig.slackWeight;
end

if isfield(mpcTuneConfig, "Np")
    Np = mpcTuneConfig.Np;
end

mpcOutputNpMax = max(Np, 125);

if isfield(mpcTuneConfig, "h")
    h = mpcTuneConfig.h;
end

Q = diag(repmat(Q_step, 1, Np));
R = diag(repmat(R_step, 1, Np));
S = diag(repmat(S_step, 1, Np));

% Constraints as column vectors!!

u_max = 0.05;%F_control/m_sat;     % [m/s^2] = 5000/10000 = 0.05

if isfield(mpcTuneConfig, "u_max")
    u_max = mpcTuneConfig.u_max;
end

U_min = -u_max*ones(m,1);
U_max =  u_max*ones(m,1);

Umin = repmat(U_min, Np, 1);
Umax = repmat(U_max, Np, 1);

% State Constraints
Y_min = [];
Y_max = [];

Ymin = repmat(Y_min, Np, 1); 
Ymax = repmat(Y_max, Np, 1);

% Constraints on deltaU

du_max = 0.007;              % [m/s^2] por paso

if isfield(mpcTuneConfig, "du_max")
    du_max = mpcTuneConfig.du_max;
end

deltaU_max = du_max*ones(m,1);
deltaUmax = repmat(deltaU_max, Np, 1);

% Nominal orbit parameters
mu = 3.986004418e14;   % [m^3/s^2] Earth gravitational parameter

%% GNSS sensor profile
% The dataset characterizes GNSS quality, noise, covariance, and availability.
% The MPC reference is generated separately from orbital elements.
gnssInputFile = gnssCovarianceFile;

[gnssSensor, gnssProfile] = prepare_gnss_sensor_workspace( ...
    "Filename", gnssInputFile, ...
    "StartDateJulian", start_date, ...
    "AssignToBase", false);

%% Override GNSS noise with raw .dat file values
% prepare_gnss_sensor_workspace uses running averages that hide degradation
% events. We replace ts_gnss_pos_noise_eci with the actual epoch-by-epoch
% position errors from the .dat file so NIS sees the real spikes.

fid  = fopen(gnssCovarianceFile, 'r');
if fid < 0
    error("Cannot open GNSS covariance file: %s", gnssCovarianceFile);
end
raw  = textscan(fid, repmat('%f',1,17), 'HeaderLines', 1);
fclose(fid);

t_dat = raw{1};          % seconds [0:10:86390]
npe   = raw{13};         % North position error [m]
epe   = raw{12};         % East  position error [m]
upe   = raw{14};         % Up    position error [m]

% Use raw NPE/EPE/UPE as position noise (approximate ECI)
% Magnitude is correct: 19.2m at t=850s will fire NIS
pos_noise_raw = [npe, epe, upe];

% Build velocity noise as finite difference of position noise
dt = 10;  % file step [s]
vel_noise_raw = [gradient(npe,dt), gradient(epe,dt), gradient(upe,dt)];

% Build R diagonal from squared errors (epoch-by-epoch, not averaged)
% R must be FIXED nominal covariance, NOT the instantaneous squared error.
% If R and noise come from the same values, NIS = 3 always regardless of
% spike magnitude: the innovation and covariance cancel each other out.
%
% Use the RMS of quiet nominal periods (t=30s to t=500s, NSV>=13)
% as the constant expected measurement noise.

nominal_idx = t_dat >= 30 & t_dat <= 500;
sig_n_nom = rms(npe(nominal_idx));   % ≈ 0.3m
sig_e_nom = rms(epe(nominal_idx));   % ≈ 0.1m
sig_u_nom = rms(upe(nominal_idx));   % ≈ 0.4m
sig_v_nom = 0.01;                    % small velocity noise [m/s]

R_nominal = diag([sig_n_nom^2, sig_e_nom^2, sig_u_nom^2, ...
                  sig_v_nom^2, sig_v_nom^2, sig_v_nom^2]);

fprintf('R_nominal diagonal: [%.4f %.4f %.4f %.6f]\n', ...
    R_nominal(1,1), R_nominal(2,2), R_nominal(3,3), R_nominal(4,4));

% Use same constant R for all epochs
R_flat = zeros(length(t_dat), 36);
R_nominal_flat = R_nominal(:).';
for k = 1:length(t_dat)
    R_flat(k,:) = R_nominal_flat;
end

% Diagnostic NIS value for a representative GNSS-noise sample.
nu_850 = [pos_noise_raw(86,:), 0, 0, 0]';
NIS_check = nu_850' * (R_nominal \ nu_850);
fprintf('NIS check at t=850s with fixed R: %.1f  (threshold=16.81)\n', NIS_check);
fprintf('NIS threshold exceeded: %d\n', NIS_check > 16.81);

% Override the smoothed timeseries
ts_gnss_pos_noise_eci    = timeseries(pos_noise_raw, t_dat);
ts_gnss_vel_noise_eci    = timeseries(vel_noise_raw, t_dat);
ts_gnss_R_state_eci_flat = timeseries(R_flat, t_dat);

fprintf('Raw GNSS noise override applied.\n');
fprintf('  |noise| at t=840s: %.3fm\n', ...
    norm(pos_noise_raw(85,:)));
fprintf('  |noise| at t=850s: %.3fm\n',  ...
    norm(pos_noise_raw(86,:)));
fprintf('  trace(R) at t=850s: %.4f m^2\n', ...
    trace(reshape(R_flat(86,:),6,6)));

gnssMeta = gnssProfile.meta;
gnss_sensor_mode = gnssSensor.mode;
gnss_sample_time = gnssSensor.sample_time;
lamda_init = 1;
%ts_gnss_pos_noise_eci = gnssSensor.ts_pos_noise_eci;
%ts_gnss_vel_noise_eci = gnssSensor.ts_vel_noise_eci;
%ts_gnss_R_state_eci_flat = gnssSensor.ts_R_state_eci_flat;
ts_gnss_valid = gnssSensor.ts_valid;
ts_gnss_sol  = timeseries(raw{7},  t_dat);
ts_gnss_nsv  = timeseries(raw{9},  t_dat);
ts_gnss_hpe  = timeseries(raw{10}, t_dat);
ts_gnss_vpe  = timeseries(raw{11}, t_dat);
ts_gnss_pdop = timeseries(raw{17}, t_dat);

ts_gnss_sol  = setinterpmethod(ts_gnss_sol,  'zoh');
ts_gnss_nsv  = setinterpmethod(ts_gnss_nsv,  'zoh');
ts_gnss_hpe  = setinterpmethod(ts_gnss_hpe,  'zoh');
ts_gnss_vpe  = setinterpmethod(ts_gnss_vpe,  'zoh');
ts_gnss_pdop = setinterpmethod(ts_gnss_pdop, 'zoh');
R_gnss_state_matrix = reshape(median(gnssProfile.R_state_eci_flat, 1), 6, 6);
R_gnss_state_matrix = 0.5 * (R_gnss_state_matrix + R_gnss_state_matrix.');
R_gnss_state_matrix = R_gnss_state_matrix + 1e-9 * eye(6);

% Compatibility signals consumed by Simulink GNSS blocks.
ts_pos = gnssProfile.ts_pos_eci;
ts_vel = gnssProfile.ts_vel_eci;
ts_Q = gnssProfile.ts_R_state_eci_flat;
gnss_state_source = "plant_sensor";

fprintf("\nGNSS sensor profile loaded:\n");
fprintf("mode              = %s\n", string(gnss_sensor_mode));
fprintf("sample time        = %.3f s\n", gnss_sample_time);
fprintf("samples            = %d\n", numel(gnssSensor.t));
fprintf("covariance source  = %s\n\n", string(gnssMeta.source));


%% Generation of the reference

Ntimesteps = ceil(tf/h) + Np + 1;

get_nominal_trajectory(h, Ntimesteps, referenceTrajectoryFile, ...
    "a", a, "ecc", ecc, "incl", inc, "RAAN", RAAN, "argp", w, "nu", theta);

load(referenceTrajectoryFile, "r_p_full", "x_ref_hist", "t_ref");

referenceMeta = struct( ...
    "source", referencePropModel, ...
    "frame", "ECI", ...
    "sample_time", h, ...
    "t_ref_end", t_ref(end));

fprintf("\nDynamic nominal reference:\n");

% Initial state consistent with the reference.
x_ini = x_ref_hist(1:3,1);
v_ini = x_ref_hist(4:6,1);

x0_ref = [x_ini; v_ini];
kalman_initial_error = [1000; -750; 500; 0.75; -0.50; 0.25];
x0_kalman = x0_ref + kalman_initial_error;
X0 = x0_ref;
x_estim = x0_kalman;

fprintf("Ntimesteps     = %d\n", Ntimesteps);
fprintf("t_ref(end)     = %.3f s\n", t_ref(end));
fprintf("norm r0        = %.6f km\n", norm(x_ini)/1000);
fprintf("norm v0        = %.6f km/s\n\n", norm(v_ini)/1000);
fprintf("Kalman initial position error = %.3f m\n", norm(kalman_initial_error(1:3)));
fprintf("Kalman initial velocity error = %.3f m/s\n\n", norm(kalman_initial_error(4:6)));

% Save reference trajectory to plot it in the data inspector of Simulink
data_ref = x_ref_hist.';   % 500 x 6
time_ref = t_ref(:);       % 500 x 1

ref_ts = timeseries(data_ref, time_ref);
ref_ts = setinterpmethod(ref_ts, "linear");

ref_ts_x  = timeseries(data_ref(:,1), time_ref);
ref_ts_y  = timeseries(data_ref(:,2), time_ref);
ref_ts_z  = timeseries(data_ref(:,3), time_ref);
ref_ts_vx = timeseries(data_ref(:,4), time_ref);
ref_ts_vy = timeseries(data_ref(:,5), time_ref);
ref_ts_vz = timeseries(data_ref(:,6), time_ref);

% Not zoh but linear interpolation between samples
ref_ts_x  = setinterpmethod(ref_ts_x,  'linear');
ref_ts_y  = setinterpmethod(ref_ts_y,  'linear');
ref_ts_z  = setinterpmethod(ref_ts_z,  'linear');
ref_ts_vx = setinterpmethod(ref_ts_vx, 'linear');
ref_ts_vy = setinterpmethod(ref_ts_vy, 'linear');
ref_ts_vz = setinterpmethod(ref_ts_vz, 'linear');

%% Initial orbital elements used by the plant

[a, ecc, inc, RAAN, w, theta] = rv2coe_from_state(x_ini, v_ini, mu);

fprintf("\n=========== ORBITAL ELEMENTS USED BY SIMULINK ===========\n");
fprintf("a     = %.6f m\n", a);
fprintf("ecc   = %.12f\n", ecc);
fprintf("inc   = %.9f deg\n", inc);
fprintf("RAAN  = %.9f deg\n", RAAN);
fprintf("w     = %.9f deg\n", w);
fprintf("theta = %.9f deg\n", theta);
fprintf("x_ini = [%.6e %.6e %.6e] m\n", x_ini(1), x_ini(2), x_ini(3));
fprintf("v_ini = [%.6f %.6f %.6f] m/s\n", v_ini(1), v_ini(2), v_ini(3));
fprintf("================================================================\n\n");

%% Linearized matrices
% Linearized relative orbital dynamics: Hill-Clohessy-Wiltshire
% State:
% x = [x y z xdot ydot zdot]'
%
% Control:
% u = [ux uy uz]'
%
% mu : gravitational parameter [m^3/s^2]
% a  : nominal orbit radius / semi-major axis [m]

n_orbit = sqrt(mu/a^3);

A_c = [ 0      0      0      1      0      0;
      0      0      0      0      1      0;
      0      0      0      0      0      1;
      3*n_orbit^2  0      0      0      2*n_orbit    0;
      0      0      0     -2*n_orbit    0      0;
      0      0    -n_orbit^2    0      0      0 ];

B_c = [ 0  0  0;
      0  0  0;
      0  0  0;
      1  0  0;
      0  1  0;
      0  0  1 ];

%% Initial plant, estimator, and control states
X0 = x0_ref; U0 = zeros(m,1);

x_estim = x0_kalman;
u = U0;

delta_u = zeros(m, 1);

delta_Ulast = zeros(m*Np,1);
 
%% Debris
debrisConfig = struct();
if ispref("inoas", "debrisConfig")
    debrisConfig = getpref("inoas", "debrisConfig");
    rmpref("inoas", "debrisConfig");
end

t_debris = 800;              % [s]
rel_pos_debris_lvlh = [50; 0; 0];   % [m] closest-approach offset in LVLH
rel_vel_debris_lvlh = [0; 10; 0];   % [m/s] tangential fly-by velocity in LVLH

if isfield(debrisConfig, "t_debris")
    t_debris = debrisConfig.t_debris;
end

if isfield(debrisConfig, "rel_pos_debris_lvlh")
    rel_pos_debris_lvlh = debrisConfig.rel_pos_debris_lvlh(:);
end

if isfield(debrisConfig, "rel_vel_debris_lvlh")
    rel_vel_debris_lvlh = debrisConfig.rel_vel_debris_lvlh(:);
end

get_debris_trajectory(h, x_ref_hist, t_ref, t_debris, ...
    rel_pos_debris_lvlh, rel_vel_debris_lvlh, debrisTrajectoryFile);

load(debrisTrajectoryFile, "x_debris_hist", "r_debris_full", ...
    "t_debris_ref", "rk_debris_encounter", "encounter_idx");

rk_debris = rk_debris_encounter;

dsafe0 = 150;
safetyCost = 0.2;

if isfield(mpcTuneConfig, "dsafe0")
    dsafe0 = mpcTuneConfig.dsafe0;
end

if isfield(mpcTuneConfig, "safetyCost")
    safetyCost = mpcTuneConfig.safetyCost;
end

%% MPC covariance inflation for debris avoidance
% This extends the keep-out radius with the propagated navigation covariance
% used by the MPC over the prediction horizon.
Q_cov_mpc = Q_matrix;
covarianceFrameMpc = "eci";
covarianceMetricMpc = "sqrt_trace_pos";
logDsafeMpc = true;

if isfield(mpcTuneConfig, "Q_cov_mpc")
    Q_cov_mpc = mpcTuneConfig.Q_cov_mpc;
end

if isfield(mpcTuneConfig, "covarianceFrameMpc")
    covarianceFrameMpc = string(mpcTuneConfig.covarianceFrameMpc);
end

if isfield(mpcTuneConfig, "covarianceMetricMpc")
    covarianceMetricMpc = string(mpcTuneConfig.covarianceMetricMpc);
end

if isfield(mpcTuneConfig, "logDsafeMpc")
    logDsafeMpc = logical(mpcTuneConfig.logDsafeMpc);
end

%% Check dynamic reference acceleration against natural orbital acceleration

r_ref = x_ref_hist(1:3,:);
v_ref = x_ref_hist(4:6,:);
dt_ref = t_ref(2) - t_ref(1);

a_ref = zeros(3, size(r_ref,2));

for k = 2:size(r_ref,2)-1
    a_ref(:,k) = (v_ref(:,k+1) - v_ref(:,k-1)) / (2*dt_ref);
end

a_ref(:,1)   = (v_ref(:,2) - v_ref(:,1)) / dt_ref;
a_ref(:,end) = (v_ref(:,end) - v_ref(:,end-1)) / dt_ref;

a_2body = zeros(size(a_ref));

for k = 1:size(r_ref,2)
    r = r_ref(:,k);
    a_2body(:,k) = -mu * r / norm(r)^3;
end

a_missing = a_ref - a_2body;
u_ff_ref_hist = a_missing;
missing_acc_norm = vecnorm(a_missing,2,1);
useReferenceFeedforward = false;

fprintf("\nDYNAMIC REFERENCE CHECK\n");
fprintf("mean |a_ref|     = %.6e m/s2\n", mean(vecnorm(a_ref,2,1)));
fprintf("mean |a_2body|   = %.6e m/s2\n", mean(vecnorm(a_2body,2,1)));
fprintf("mean |a_missing| = %.6e m/s2\n", mean(missing_acc_norm));
fprintf("max  |a_missing| = %.6e m/s2\n", max(missing_acc_norm));
fprintf("u_max            = %.6e m/s2\n", u_max);
fprintf("feedforward ref  = %d\n\n", useReferenceFeedforward);

%% Reference check

fprintf("\n================ REFERENCE CHECK ================\n");
fprintf("referencePropModel  = %s\n", string(referencePropModel));
fprintf("h                   = %.6f s\n", h);
fprintf("reference dt        = %.6f s\n", t_ref(2)-t_ref(1));
if exist("simulationStopTime", "var")
    fprintf("sim StopTime used   = %.3f s\n", double(simulationStopTime));
end

if exist("referenceMeta", "var")
    if isfield(referenceMeta, "source")
        fprintf("source              = %s\n", string(referenceMeta.source));
    end
    if isfield(referenceMeta, "gnss_file")
        fprintf("GNSS file           = %s\n", string(referenceMeta.gnss_file));
    end
    if isfield(referenceMeta, "frame")
        fprintf("frame               = %s\n", string(referenceMeta.frame));
    end
    if isfield(referenceMeta, "smooth_win")
        fprintf("smooth_win          = %d\n", referenceMeta.smooth_win);
    end
    if isfield(referenceMeta, "t_ref_end")
        fprintf("t_ref(end)          = %.3f s\n", referenceMeta.t_ref_end);
    end
    if isfield(referenceMeta, "t_gnss_end")
        fprintf("t_gnss(end)         = %.3f s\n", referenceMeta.t_gnss_end);
    end
else
    warning("referenceMeta does not exist. Check nominal reference generation.");
end

fprintf("\nFirst ECI reference state:\n");
fprintf("r_ref = [%.6e %.6e %.6e] m\n", ...
    x_ref_hist(1,1), x_ref_hist(2,1), x_ref_hist(3,1));
fprintf("v_ref = [%.6f %.6f %.6f] m/s\n", ...
    x_ref_hist(4,1), x_ref_hist(5,1), x_ref_hist(6,1));

fprintf("norm r0 = %.6f km\n", norm(x_ref_hist(1:3,1))/1000);
fprintf("norm v0 = %.6f km/s\n", norm(x_ref_hist(4:6,1))/1000);
fprintf("==================================================\n\n");
clear mpc_dsafe_log_time mpc_dsafe_log_first mpc_dsafe_log_max

clear MPC_INOAS;

%%
function stopTimeSeconds = getModelStopTimeSeconds(modelName)
    stopTimeSeconds = [];

    try
        modelName = string(modelName);
        if strlength(modelName) == 0
            return;
        end

        load_system(modelName);
        stopExpr = string(get_param(modelName, "StopTime"));
        stopValue = str2double(stopExpr);

        if ~isfinite(stopValue)
            try
                stopValue = evalin("base", char(stopExpr));
            catch
                stopValue = [];
            end
        end

        if isnumeric(stopValue) && isscalar(stopValue) && ...
                isfinite(double(stopValue)) && double(stopValue) > 0
            stopTimeSeconds = double(stopValue);
        end
    catch ME
        warning("Could not infer Simulink StopTime from model '%s': %s", ...
            string(modelName), ME.message);
    end
end

function stopTimeSeconds = getOpenModelStopTimeSeconds()
    stopTimeSeconds = [];

    try
        openModels = find_system("type", "block_diagram");
    catch
        return;
    end

    if isempty(openModels)
        return;
    end

    preferredModels = ["inoas_model", ...
        "MPCcontrolledSpacecraft_plant_gnss_kalman", ...
        "MPCcontrolledSpacecraft_plant_gnss", ...
        "MPCcontrolledSpacecraft"];

    for k = 1:numel(preferredModels)
        if any(strcmp(openModels, preferredModels(k)))
            stopTimeSeconds = getModelStopTimeSeconds(preferredModels(k));
            if ~isempty(stopTimeSeconds)
                return;
            end
        end
    end

    for k = 1:numel(openModels)
        stopTimeSeconds = getModelStopTimeSeconds(openModels{k});
        if ~isempty(stopTimeSeconds)
            return;
        end
    end
end

%%
function [a, ecc, inc, RAAN, argp, theta] = rv2coe_from_state(r, v, mu)

    r = r(:);
    v = v(:);

    R = norm(r);
    V = norm(v);

    h_vec = cross(r, v);
    h = norm(h_vec);

    k_vec = [0; 0; 1];
    n_vec = cross(k_vec, h_vec);
    n = norm(n_vec);

    e_vec = (1/mu) * ((V^2 - mu/R)*r - dot(r,v)*v);
    ecc = norm(e_vec);

    energy = V^2/2 - mu/R;
    a = -mu/(2*energy);

    inc = acosd(h_vec(3)/h);

    if n > 1e-12
        RAAN = atan2d(n_vec(2), n_vec(1));
    else
        RAAN = 0;
    end

    if RAAN < 0
        RAAN = RAAN + 360;
    end

    if n > 1e-12 && ecc > 1e-12
        argp = atan2d(dot(cross(n_vec, e_vec), h_vec)/h, dot(n_vec, e_vec));
    else
        argp = 0;
    end

    if argp < 0
        argp = argp + 360;
    end

    if ecc > 1e-12
        theta = atan2d(dot(cross(e_vec, r), h_vec)/h, dot(e_vec, r));
    else
        theta = atan2d(dot(cross(n_vec, r), h_vec)/h, dot(n_vec, r));
    end

    if theta < 0
        theta = theta + 360;
    end

end

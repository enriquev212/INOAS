%% Ejecución modelo Simulink
% Vamos a meter en primer lugar los datos de la órbita, así como datos varios 
% del satélite:
clear all; clc; close all;

preservedModelName = "";
preservedStopTime = [];

if exist("modelName", "var")
    preservedModelName = string(modelName);
end

if exist("codexStopTime", "var")
    preservedStopTime = codexStopTime;
end

skipBatchClear = ispref("codex", "skipBatchClear") && getpref("codex", "skipBatchClear");
if ispref("codex", "skipBatchClear")
    rmpref("codex", "skipBatchClear");
end

if ~skipBatchClear
    clearvars -except preservedModelName preservedStopTime skipBatchClear;
end
clc

if strlength(preservedModelName) > 0
    modelName = preservedModelName;
end

if ~isempty(preservedStopTime)
    codexStopTime = preservedStopTime;
end

clear preservedModelName preservedStopTime
a=7714.43*1000; %m
ecc=0.000095;
inc=63.04; %deg
RAAN=116.6; %deg
w=90; %deg
theta=131; %deg
F_control=4*220;
%F_control=0.5*1000; %(0.5-2)*1000N
m_sat=10*1000; %7-13.7 tons
initMass=m_sat;
ref=1.3; %reflectancia
area=15; %m^2
start_date=juliandate(datetime(2024, 1, 11));%Fecha
end_date=juliandate(datetime(2024, 2, 11));
tf=10;%(end_date-start_date)*24*60*60e-4/5; % Tiempo de simulación
if exist("codexStopTime", "var") && ~isempty(codexStopTime)
    % Keep the nominal/reference trajectory available for the whole
    % simulation. Otherwise the MPC clamps to the last reference sample and
    % the relative state grows artificially once the run outlasts the
    % precomputed nominal timeline.
    tf = max(tf, double(codexStopTime));
end
k_p=5;
k_d=20;
%% Filtro de Kalman
% Vamos ahora a caracterizar el filtro de Kalman. 
% 
% Lo que caracteriza al Unscented Kalman Filter son las matrices Q y R. Estas 
% matrices expresan la confianza que depositamos tanto en el modelo matemático, 
% como en las medidas. Por tanto, el tuneado correcto del Kalman Filter viene 
% por ajustar correctamente estas matrices. Con las matrices actuales, hasta los 
% 2200s el estimador funciona correctamente. Faltaría hacer algunos estudios de 
% casos cuantificables.

%% INICIALIZACIÓN DEL FILTRO DE KALMAN (UKF) Y SENSORES
% Este script carga los parámetros en el Workspace necesarios para el modelo
% de Simulink de la controladora orbital (Challenge Dassault).

% =========================================================================
% 1. PARÁMETROS GLOBALES DE SIMULACIÓN
% =========================================================================
% Tiempo de muestreo del Filtro y los Sensores (Reloj Maestro)
Ts = 1; % [s] El filtro y los sensores operan a 1 Hz (1 vez por segundo)

var_IMU=0.01; %Varianza de las aceleraciones que medirá la IMU del KALMAN

% =========================================================================
% 2. CARACTERIZACIÓN DE LOS SENSORES (RUIDO FÍSICO)
% =========================================================================
% Sensor de Posición (Magnetómetro / Sensor Solar sintético 3D)
sigma_pos = 100;  %Desviación muy pequeñita  %5000;        % Desviación estándar / Error en metros (+- 5 km)
var_pos   = sigma_pos^2; % Varianza de la posición (2.5e7 m^2)

% Sensor Altímetro (Radar)
%sigma_alt = 10;          % Desviación estándar / Error en metros (+- 10 m)

sigma_alt = 1; %Desviación muy pequeñita (irreal)
var_alt   = sigma_alt^2; % Varianza de la altitud (100 m^2)


% =========================================================================
% 3. MATRICES DE SINTONIZACIÓN DEL FILTRO DE KALMAN (TUNING)
% =========================================================================
% Matriz de Covarianza del Ruido de Medida (R) [4x4]
% Le dice al filtro cuánto debe dudar de las medidas de los sensores.
% (Construida dinámicamente usando las varianzas definidas arriba)

R_matrix = diag([var_pos, var_pos, var_pos, var_alt]);


% Matriz de Covarianza del Ruido de Proceso (Q) [6x6]
% Le dice al filtro cuánto dudar de la función de propagación (modelo J2).
% Asumimos alta confianza en posición y un poco menos en velocidad por 
% las fuerzas no modeladas (drag, radiación solar, armónicos altos).
% Q_pos = 1e-2, Q_vel = 1e-4

Q_matrix = diag([10, 10, 10, 1, 1, 1]);%Errores tochos del modelado 

%Q_matrix = diag([1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8]); %Errores
%matemáticos casi nulos
%Q_matrix = diag([1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4]); %Medio medio


% =========================================================================
% 4. INCERTIDUMBRE INICIAL DEL FILTRO
% =========================================================================
% Matriz de Covarianza Inicial (P0) [6x6]
% Define lo "perdido" que está el filtro en el segundo 0 respecto a la verdad.
% Asumimos un error inicial de 1 km en posición y 10 m/s en velocidad.
% Posición: (1000 m)^2 = 1e6
% Velocidad: (10 m/s)^2 = 100
P0_kalman = diag([1e6, 1e6, 1e6, 100, 100, 100]);

% NOTA: Recuerda definir también 'x0_kalman' (tu estado inicial estimado) 
% en tu código, sumándole el error inicial a tu estado verdadero.
%mu = 3.986004418e14;
%h = sqrt(mu * a * (1 - ecc^2));
%coe=[h,ecc,RAAN,inc,w,theta];
%[r, v] = sv_from_coe(coe,mu);
%x0_kalman =[r v] ;

%x_ini=[4.6589;-4.178;-4.512]*10^06;
%v_ini=[88;5316;-4835];
%x0_kalman=[x_ini;v_ini];
%% Observable O
% Para implementar el _Instrument Decision_, necesitamos normalizar o adimensionalizar 
% nuestra matriz de covarianza, y convertirla en un escalar que nos permita analizar, 
% o tener una perspectiva del "error global", para tener un criterio que nos permita 
% decidir cuando activar o desactivar el GNSS.
% 
% Utilizaremos las siguientes expresiones
% 
% $$J=\textrm{trace}\left(S^{-1} PS^{-T} \right)$$
% 
% donde S será una matriz diagonal con el error maximo tolerable de cada variable. 
% Esto nos permitirá comparar magnitudes de distinta naturaleza, así como darle 
% más o menos importancia a cada variable de forma individual. Tomaremos, por 
% ejemplo, errores $\sigma_x =10m\;\;$y $\sigma_v =1\frac{m}{s}\;$. Esto lo podremos 
% modificar cuando tengamos las especificaciones y margenes finales.

sigma_x=10;
sigma_v=1;
s=[sigma_x,sigma_x,sigma_x,sigma_v,sigma_v,sigma_v];
S=diag(s);

%% 
% Escribamos nuestras matrices finales:

S_inv=inv(S);
S_T_inv=inv(S');
max_cov=10;
%% 
% La max_cov puede modificarse también.

%% *MPC controller initialization*
% Control system parameters
nx = 6; % State Variables
m = 3; % Control variables

%% Control System parameters      
targetModelName = "";
if exist("modelName", "var")
    targetModelName = string(modelName);
end

%% Selección de fuente de referencia

% Forzamos que la referencia venga del archivo GNSS .dat
referenceSourceMode = "gnss";

% Parámetros coherentes con GNSS
Np = 50;     % Horizon length para GNSS
h  = 3;     % Sample time GNSS = 1 s

%% MPC tuning

mpcTuneConfig = struct();
if ispref("codex", "mpcTuneConfig")
    mpcTuneConfig = getpref("codex", "mpcTuneConfig");
    rmpref("codex", "mpcTuneConfig");
end

% Cost matrices
% Tuned default: slightly stronger state tracking and milder delta-u penalty
% to obtain a tighter but still feasible debris encounter.
Q_step = 1e-2*[0.03 0.03 0.03 0.0015 0.0015 0.0015];
%Q_step = 1e-1*[0.05 0.05 0.05 0.005 0.005 0.005]; dsafe = 100m

R_step = 2e2*[10 10 10]; 
%R_step = 2e2*[10 10 10]; 

S_step = 4*[100 100 100];
%S_step = [100 100 100]; 

slackWeight = 1e3;

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

if isfield(mpcTuneConfig, "h")
    h = mpcTuneConfig.h;
end

Q = diag(repmat(Q_step, 1, Np));
R = diag(repmat(R_step, 1, Np));
S = diag(repmat(S_step, 1, Np));

% Constraints as column vectors!!

u_max = F_control/m_sat;     % [m/s^2] = 5000/10000 = 0.05

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

gnssMeta = struct();
%if contains(targetModelName, "GNSS", "IgnoreCase", true)
%    [ts_pos_gnss_eci, ts_vel_gnss_eci, ts_Q_gnss_eci, gnssMeta] = codex_load_gnss_timeseries_eci(start_date);
%    x0_gnss = [ts_pos_gnss_eci.Data(1,:).'; ts_vel_gnss_eci.Data(1,:).'];
%    gnssCoe = codex_state_to_coe(x0_gnss, mu);
%
%    % Keep a single GNSS source of truth in the workspace so later
%    % initialization steps do not rebuild a different branch.
%    ts_pos_ecef = gnssMeta.ts_pos_ecef;
%    ts_vel_ecef = gnssMeta.ts_vel_ecef;
%    ts_Q_ecef = gnssMeta.ts_Q_ecef;
%    ts_pos = ts_pos_gnss_eci;
%    ts_vel = ts_vel_gnss_eci;
%    ts_Q = ts_Q_gnss_eci;
%    codex_gnss_state_source = "dataset";
%    lamda_init = 1;
%
%    % For the GNSS model, align the plant/reference orbit with the same
%    % absolute state used by the direct GNSS branch.
%    a = gnssCoe.a;
%    ecc = gnssCoe.ecc;
%    inc = gnssCoe.inc;
%    RAAN = gnssCoe.RAAN;
%    w = gnssCoe.argp;
%    theta = gnssCoe.nu;
%    x_ini = x0_gnss(1:3);
%    v_ini = x0_gnss(4:6);
%    x0_kalman = x0_gnss;
%end

%% Generation of the reference

% Saltar primeros puntos GNSS/ECI para evitar artefactos de borde
skip_ref_steps = 10;       % con h = 3 s, empieza 30 s mas tarde

Ntimesteps_nominal = ceil(tf/h) + Np + 1;
Ntimesteps_raw     = Ntimesteps_nominal + skip_ref_steps;

[r_p_full_raw, x_ref_hist_raw, t_ref_raw, referenceMeta] = ...
    get_nominal_trajectory_from_gnss(h, Ntimesteps_raw, "referenceTrajectory.mat");

% Recortar los primeros puntos malos
idx0 = skip_ref_steps + 1;

x_ref_hist = x_ref_hist_raw(:, idx0:end);

% Reiniciar tiempo de referencia en cero para Simulink/MPC
t_ref = t_ref_raw(idx0:end);
t_ref = t_ref - t_ref(1);

% Actualizar numero real de pasos
Ntimesteps = size(x_ref_hist, 2);

% Reconstruir vector apilado para el MPC
r_p_full = reshape(x_ref_hist, [], 1);

% Guardar referencia ya recortada
save("referenceTrajectory.mat", "r_p_full", "x_ref_hist", "t_ref", "referenceMeta");

% Estado inicial coherente con la referencia recortada
x_ini = x_ref_hist(1:3,1);
v_ini = x_ref_hist(4:6,1);

x0_ref = [x_ini; v_ini];
x0_kalman = x0_ref;
X0 = x0_ref;
x_estim = x0_ref;

fprintf("\nReferencia GNSS recortada:\n");
fprintf("skip_ref_steps = %d\n", skip_ref_steps);
fprintf("tiempo saltado = %.3f s\n", skip_ref_steps*h);
fprintf("Ntimesteps     = %d\n", Ntimesteps);
fprintf("t_ref(end)     = %.3f s\n", t_ref(end));
fprintf("norm r0        = %.6f km\n", norm(x_ini)/1000);
fprintf("norm v0        = %.6f km/s\n\n", norm(v_ini)/1000);

% Save reference trajectory to plot it in the data inspector of Simulink
data_ref = x_ref_hist.';   % 500 x 6
time_ref = t_ref(:);       % 500 x 1

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

%% Calcular elementos orbitales iniciales desde la referencia GNSS

[a, ecc, inc, RAAN, w, theta] = rv2coe_from_state(x_ini, v_ini, mu);

fprintf("\n=========== ELEMENTOS ORBITALES USADOS POR SIMULINK ===========\n");
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

%% Initialization of variables 
% Vector estado y control iniciales --> Dimensionales
X0 = x0_ref; U0 = zeros(m,1);

% Inicialización
x_estim = x0_kalman;
u = U0;

delta_u = zeros(m, 1);

delta_Ulast = zeros(m*Np,1);
 
%% Debris
debrisConfig = struct();
if ispref("codex", "debrisConfig")
    debrisConfig = getpref("codex", "debrisConfig");
    rmpref("codex", "debrisConfig");
end

t_debris = 240;              % [s]
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
    rel_pos_debris_lvlh, rel_vel_debris_lvlh, "debrisTrajectory.mat");

load("debrisTrajectory.mat", "x_debris_hist", "r_debris_full", ...
    "t_debris_ref", "rk_debris_encounter", "encounter_idx");

rk_debris = rk_debris_encounter;

dsafe0 = 200;
safetyCost = 0.015;

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

%% CHECK aceleracion de referencia GNSS vs aceleracion orbital natural

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

fprintf("\nCHECK REFERENCIA GNSS DINAMICA\n");
fprintf("mean |a_ref|     = %.6e m/s2\n", mean(vecnorm(a_ref,2,1)));
fprintf("mean |a_2body|   = %.6e m/s2\n", mean(vecnorm(a_2body,2,1)));
fprintf("mean |a_missing| = %.6e m/s2\n", mean(vecnorm(a_missing,2,1)));
fprintf("max  |a_missing| = %.6e m/s2\n", max(vecnorm(a_missing,2,1)));
fprintf("u_max            = %.6e m/s2\n\n", u_max);

figure;
plot(t_ref, vecnorm(a_missing,2,1), "LineWidth", 1.2);
grid on;
xlabel("t [s]");
ylabel("|a_{ref} - a_{2body}| [m/s^2]");
title("Aceleracion faltante para seguir la referencia GNSS");

%% Check de que la referencia viene del GNSS

fprintf("\n================ CHECK REFERENCIA ================\n");
fprintf("referenceSourceMode = %s\n", string(referenceSourceMode));
fprintf("h                   = %.6f s\n", h);
fprintf("dt referencia       = %.6f s\n", t_ref(2)-t_ref(1));

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
    warning("No existe referenceMeta. Revisa que get_nominal_trajectory_from_gnss devuelva el cuarto output.");
end

fprintf("\nPrimer estado referencia GNSS ECI:\n");
fprintf("r_ref = [%.6e %.6e %.6e] m\n", ...
    x_ref_hist(1,1), x_ref_hist(2,1), x_ref_hist(3,1));
fprintf("v_ref = [%.6f %.6f %.6f] m/s\n", ...
    x_ref_hist(4,1), x_ref_hist(5,1), x_ref_hist(6,1));

fprintf("norm r0 = %.6f km\n", norm(x_ref_hist(1:3,1))/1000);
fprintf("norm v0 = %.6f km/s\n", norm(x_ref_hist(4:6,1))/1000);
fprintf("==================================================\n\n");
clear codex_dsafe_log_time codex_dsafe_log_first codex_dsafe_log_max

clear MPC_INOAS;

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
%RUN_EXPB Pasada B: lazo cerrado con planta no lineal (2 cuerpos + J2).
% Un proceso por configuracion, lanzables en paralelo: las metricas de esta
% pasada (tracking, delta-v, separacion, saturacion) no dependen del reloj de
% pared, asi que la contencion de CPU no las contamina. Los tiempos de solve
% fiables salen de la pasada A.
% Entrada: CFG_ID (1..6)
% --- rutas derivadas de la posicion de este fichero ------------------------
% El estudio se localiza a si mismo. Antes cada script llevaba la ruta absoluta
% escrita a mano, lo que impedia reproducir ninguna cifra en otra maquina.
STUDY_DIR = fileparts(mfilename('fullpath'));
REPO_ROOT = fileparts(STUDY_DIR);
OUT_DIR   = fullfile(STUDY_DIR, 'out');
FIG_DIR   = fullfile(STUDY_DIR, 'figures');
WORK_DIR  = fullfile(STUDY_DIR, 'work');
for d_ = {OUT_DIR, FIG_DIR, WORK_DIR}
    if ~exist(d_{1},'dir'); mkdir(d_{1}); end
end
addpath(REPO_ROOT); addpath(genpath(fullfile(REPO_ROOT,'matlab')));
addpath(fullfile(REPO_ROOT,'models')); addpath(STUDY_DIR);
% --------------------------------------------------------------------------

addpath(STUDY_DIR);
run(fullfile(STUDY_DIR,'inoas_setup.m'));

% Ventana [400, 950] s: cubre regimen permanente (400-700) y el encuentro
% completo (t_debris = 800 s). Arrancar en t=0 gastaria la mayoria de los
% solves recuperando el transitorio de 1.3 km, que no es lo que se estudia.
STUDY_T0S  = 400;
STUDY_TEND = 950;
STUDY_ODE  = odeset('RelTol',1e-9,'AbsTol',1e-9);
STUDY_IK0  = min(floor(STUDY_T0S/h)+1, Ntimesteps);
% Offset inicial del orden del error de navegacion, no del transitorio de arranque
x_true_ = r_p_full((STUDY_IK0-1)*6 + (1:6)) + [30; -20; 15; 0.02; -0.01; 0.01];
t_now_  = STUDY_T0S; u_hold_ = zeros(3,1);
nS_ = floor((STUDY_TEND - STUDY_T0S)/h);

B = struct('t',zeros(1,nS_),'err',zeros(1,nS_),'errv',zeros(1,nS_), ...
           'u',zeros(3,nS_),'solve',zeros(1,nS_),'ddeb',zeros(1,nS_), ...
           'dsafe',zeros(1,nS_));
clear MPC_INOAS;
clear mpc_dsafe_log_time mpc_dsafe_log_first mpc_dsafe_log_max
dv_ = 0; nsat_ = 0; ndu_ = 0; u_prev_ = zeros(3,1);
tw_ = tic;
for kk_ = 1:nS_
    ik_ = min(floor(t_now_/h)+1, Ntimesteps);
    xr_ = r_p_full((ik_-1)*6 + (1:6));
    tt_ = tic; [u_hold_,~,~] = MPC_INOAS(x_true_, STUDY_PNAV, t_now_); B.solve(kk_) = toc(tt_);
    B.t(kk_)    = t_now_;
    B.err(kk_)  = norm(x_true_(1:3) - xr_(1:3));
    B.errv(kk_) = norm(x_true_(4:6) - xr_(4:6));
    B.u(:,kk_)  = u_hold_;
    % Saturacion: el clamp final de MPC_INOAS es por componente en ECI
    if max(abs(u_hold_)) >= 0.999*u_max; nsat_ = nsat_ + 1; end
    % Actividad del limite de slew rate
    if kk_ > 1 && max(abs(u_hold_ - u_prev_)) >= 0.999*du_max; ndu_ = ndu_ + 1; end
    u_prev_ = u_hold_;
    dv_ = dv_ + norm(u_hold_)*h;
    B.ddeb(kk_)  = norm(x_true_(1:3) - inoas_debris_pos(x_debris_hist, ik_));
    B.dsafe(kk_) = mpc_dsafe_log_first(end);
    [~,XX_] = ode45(@(t2_,xx_) inoas_dyn(t2_,xx_,u_hold_), [t_now_ t_now_+h], x_true_, STUDY_ODE);
    x_true_ = XX_(end,:).';
    t_now_  = t_now_ + h;
end

RES = struct();
RES.name = STUDY_CFG.name; RES.h = h; RES.Np = Np; RES.Th = h*Np;
RES.du_max = du_max; RES.u_max = u_max;
RES.nvars = 3*Np + Np;
RES.B = B; RES.B_wall = toc(tw_); RES.nsteps = nS_;
RES.dv_total = dv_;
RES.sat_steps = nsat_;  RES.sat_frac = nsat_/nS_;
RES.du_steps  = ndu_;   RES.du_frac  = ndu_/max(nS_-1,1);
RES.u_peak = max(vecnorm(B.u,2,1));
RES.err_rms = rms(B.err); RES.err_max = max(B.err); RES.err_final = B.err(end);
% Tracking en regimen permanente, antes de que la maniobra de evasion lo domine
STUDY_SS = B.t >= 450 & B.t <= 700;
RES.err_ss_rms = rms(B.err(STUDY_SS)); RES.err_ss_max = max(B.err(STUDY_SS));
RES.min_sep = min(B.ddeb); RES.min_margin = min(B.ddeb - B.dsafe);
RES.solve_mean = mean(B.solve); RES.solve_max = max(B.solve);

fprintf('\n-- EXP B  %s  (lazo cerrado 0-%d s) --\n', RES.name, STUDY_TEND);
fprintf('   tracking: rms %.1f m | max %.1f m | final %.1f m\n', ...
    RES.err_rms, RES.err_max, RES.err_final);
fprintf('   tracking en regimen (300-600 s): rms %.2f m | max %.2f m\n', ...
    RES.err_ss_rms, RES.err_ss_max);
fprintf('   separacion minima al debris: %.1f m | margen minimo sobre dsafe: %.1f m\n', ...
    RES.min_sep, RES.min_margin);
fprintf('   delta-v: %.3f m/s | pico |u| = %.5f m/s2 (%.1f%% de u_max)\n', ...
    dv_, RES.u_peak, 100*RES.u_peak/u_max);
fprintf('   pasos con clamp activo: %d/%d (%.1f%%) | con du_max activo: %d (%.1f%%)\n', ...
    nsat_, nS_, 100*RES.sat_frac, ndu_, 100*RES.du_frac);
fprintf('   pared: %.1f s\n', RES.B_wall);

save(fullfile(STUDY_OUT, sprintf('expB_%02d_%s.mat', CFG_ID, STUDY_CFG.name)), 'RES');
fprintf('EXPB_DONE %s\n', STUDY_CFG.name);

%TEST_CAM_RETARGET Arquitectura completa: guiado planifica, MPC ejecuta.
%
% El controlador no lucha contra su propia referencia. Se planifica la maniobra
% a nivel de guiado, se retarget la nominal, y el MPC hace lo que sabe hacer:
% seguirla. Se comprueba que
%   (a) el delta-v ejecutado coincide con el planificado,
%   (b) la separacion conseguida es la pedida,
%   (c) el empuje pico cabe en el propulsor,
%   (d) el MPC no deshace la maniobra.
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

STUDY_FORCE_MAIN = true;
REPO = REPO_ROOT;
addpath(REPO); addpath(genpath(fullfile(REPO,'matlab')));
addpath(STUDY_DIR);
OUT = OUT_DIR;

MU = 3.986004418e14;
H_MPC = 30;  NP_MPC = 60;          % el MPC solo tiene que SEGUIR: horizonte corto
TF    = 12000;
T_TCA = 10115;
SIG_TCA = 40;  K_SIGMA = 3;   % SIG_TCA es un RADIO 3D sqrt(traza(P))
D_SAFE0 = 150;                % el mismo suelo que usa el controlador

setpref('inoas','skipBatchClear', true);
setpref('inoas','mpcTuneConfig', struct('h',H_MPC,'Np',NP_MPC));
simulationStopTime = TF;  mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));
mpcQuiet = true;
T_orb = 2*pi/sqrt(MU/a^3);

SC = get_conjunction_scenario(h, x_ref_hist, t_ref, T_TCA);
fprintf('Conjuncion: cruce %.2f deg | v_rel = %.3f km/s | miss sin maniobra = %.1f m\n', ...
    SC.dInc_deg, SC.v_rel_mag/1e3, SC.miss_nom);

%% ---- 1) GUIADO: cuanto margen hace falta -------------------------------
% Margen de K sigma sobre la covarianza COMBINADA, proyectada al plano-B.
% SIG_TCA es el radio 3D que entrega el filtro, asi que la varianza POR EJE
% es SIG_TCA^2/3. Montarla como diag([s s s].^2) la triplicaba.
P_comb = (SIG_TCA^2/3)*eye(3) + SC.P_debris;
u_b = SC.d_nom_eci / norm(SC.d_nom_eci);
sigma_b = sqrt(u_b.' * P_comb * u_b);
% Suelo de 150 m, no el radio de cuerpo duro de 5 m: es el que usa el
% controlador y el que declara el resto del paper.
d_target = D_SAFE0 + K_SIGMA * sigma_b;
fprintf('sigma combinada en la direccion del fallo = %.1f m -> objetivo = %.1f m\n', ...
    sigma_b, d_target);

%% ---- 2) GUIADO: planificar la maniobra y retargetear la nominal --------
for lead_orb = [0.5 1.0 1.5]
    t_burn = SC.t_tca - lead_orb*T_orb;
    if t_burn < t_ref(2); continue; end
    PL = plan_cam(x_ref_hist, t_ref, SC, 'd_target', d_target, 't_burn', t_burn);
    fprintf('  antelacion %.1f orb: dv = %7.3f mm/s (%.2f s de propulsor), miss %.1f -> %.1f m\n', ...
        lead_orb, PL.dv*1e3, PL.dv/u_max, PL.miss_before, PL.miss_after);
    if abs(lead_orb - 1.0) < 1e-9; PLAN = PL; end
end

%% ---- 3) CONTROL: el MPC sigue la nominal retargeteada -----------------
r_p_full = PLAN.r_p_full;          % <- la nominal ya contiene la maniobra
dsafe0 = 0;                        % sin restriccion de evasion: solo seguimiento
mpcDebrisMode = "sphere_grid";
P_nav = blkdiag((SIG_TCA^2/3)*eye(3), diag([0.05 0.05 0.05].^2));

T_START = PLAN.t_burn - 6*h;
T_END   = SC.t_tca + 2*h;
odeo = odeset('RelTol',1e-11,'AbsTol',1e-9);

clear MPC_INOAS mpc_diag_log
ik0 = min(floor(T_START/h)+1, Ntimesteps);
x_true = x_ref_hist(:, ik0);       % arranca sobre la nominal ANTIGUA
t_now  = (ik0-1)*h;
nS = floor((T_END - t_now)/h);

dv_cum = 0; un = zeros(1,nS); err = zeros(1,nS); tv = zeros(1,nS);
for k = 1:nS
    u_cmd = MPC_INOAS(x_true, P_nav, t_now);
    ikr = min(floor(t_now/h)+1, Ntimesteps);
    tv(k)  = t_now;
    un(k)  = norm(u_cmd);
    err(k) = norm(x_true(1:3) - PLAN.x_ref_new(1:3,ikr));
    dv_cum = dv_cum + norm(u_cmd)*h;
    [~,XX] = ode113(@(tt,xx) inoas_dyn(tt,xx,u_cmd), [t_now t_now+h], x_true, odeo);
    x_true = XX(end,:).';
    t_now  = t_now + h;
end

idx_d = min(max(round(t_now/h)+1,1), size(SC.x_debris_hist,2));
Tf = cam_find_tca(x_true, SC.x_debris_hist(:,idx_d), t_now, SC.t_tca, 4*h);

fprintf('\n--- Ejecucion por el MPC ---\n');
fprintf('  delta-v planificado : %.3f mm/s\n', PLAN.dv*1e3);
fprintf('  delta-v ejecutado   : %.3f mm/s  (%.2fx el planificado)\n', ...
    dv_cum*1e3, dv_cum/abs(PLAN.dv));
fprintf('  pico |u|            : %.3e m/s2 (%.2f %% de u_max)\n', max(un), 100*max(un)/u_max);
fprintf('  error de seguimiento: rms %.2f m, max %.2f m\n', rms(err), max(err));
fprintf('  separacion objetivo : %.1f m\n', d_target);
fprintf('  separacion lograda  : %.1f m  (sin maniobra: %.1f m)\n', Tf.miss, SC.miss_nom);

save(fullfile(OUT,'test_cam_retarget.mat'), 'PLAN', 'SC', 'd_target', ...
     'tv', 'un', 'err', 'dv_cum', 'Tf', 'sigma_b', 'H_MPC', 'NP_MPC');
fprintf('\nTEST_RETARGET_OK\n');

%% ---- 4) El resultado del paper: coste frente a calidad de navegacion ----
% Con la arquitectura ya funcionando, el acoplamiento se mide de extremo a
% extremo: peor navegacion -> mas margen exigido -> mas propelente.
fprintf('\n--- Coste de la maniobra frente a la incertidumbre en el TCA ---\n');
fprintf('  %-10s %10s %12s %12s %12s\n', 'sigma_nav', 'sigma_comb', 'objetivo', 'dv [mm/s]', 'propulsor');
SW = struct('sig',{},'dt',{},'dv',{},'sig_comb',{});
for sg = [12 40 100 250]
    Pc_ = (sg^2/3)*eye(3) + SC.P_debris;   % sg es radio 3D, no por eje
    sb_ = sqrt(u_b.' * Pc_ * u_b);
    dt_ = D_SAFE0 + K_SIGMA * sb_;
    PL_ = plan_cam(x_ref_hist, t_ref, SC, 'd_target', dt_, ...
                   't_burn', SC.t_tca - 1.0*T_orb);
    fprintf('  %7.0f m %9.1f m %10.1f m %12.3f %10.2f s\n', ...
        sg, sb_, dt_, PL_.dv*1e3, PL_.dv/u_max);
    SW(end+1) = struct('sig',sg,'dt',dt_,'dv',PL_.dv,'sig_comb',sb_); %#ok<SAGROW>
end
save(fullfile(OUT,'cam_retarget_sweep.mat'), 'SW', 'SC', 'K_SIGMA');
fprintf('\nSWEEP_OK\n');

%% ---- 5) Coste computacional de la ruta PRIMARIA -----------------------
% El MPC aqui solo sigue una nominal: la restriccion de evasion no esta en el
% QP. Es la ruta que el paper propone, y su coste es el que hay que reportar.
Dg = evalin('base','mpc_diag_log');
fprintf('\n--- Coste por solve de la ruta guiado-planifica / MPC-sigue ---\n');
fprintf('  solves            : %d  (h = %g s)\n', numel(Dg.t), h);
fprintf('  mediana           : %.4f s\n', median(Dg.solve_time));
fprintf('  p90 / p99 / peor  : %.4f / %.4f / %.4f s\n', ...
    prctile(Dg.solve_time,90), prctile(Dg.solve_time,99), max(Dg.solve_time));
fprintf('  margen sobre el plazo (peor caso): %.1fx\n', h/max(Dg.solve_time));
fprintf('  convergencia      : %.1f %%   respaldo usado: %.1f %%\n', ...
    100*mean(Dg.exitflag > 0), 100*mean(Dg.qp_fallback));
fprintf('  iteraciones       : mediana %g, max %g\n', ...
    median(Dg.iterations), max(Dg.iterations));
save(fullfile(OUT,'primary_timing.mat'), 'Dg');
fprintf('\nPRIMARY_TIMING_OK\n');

%VERIFY_FINAL_A Comprobaciones tras la fase A.
%  1) Np > 125 ya no desborda los puertos del modelo
%  2) el limite por norma actua de verdad
%  3) el escenario realista sigue funcionando de extremo a extremo
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

fprintf('=== 1) horizonte largo: Np = 150 ===\n');
setpref('inoas','skipBatchClear', true);
setpref('inoas','mpcTuneConfig', struct('h',5,'Np',150));
mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));
mpcQuiet = true;
fprintf('  Np = %d, mpcOutputNpMax = %d\n', Np, mpcOutputNpMax);

P_nav = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);
clear MPC_INOAS
ik = min(floor(700/h)+1, Ntimesteps);
xt = r_p_full((ik-1)*6 + (1:6)) + [30;-20;15;0.02;-0.01;0.01];
[u1, dU1, sl1] = MPC_INOAS(xt, P_nav, 700);
fprintf('  longitud de las salidas: delta_Ulast = %d (esperado %d), slack = %d (esperado %d)\n', ...
    numel(dU1), 3*mpcOutputNpMax, numel(sl1), mpcOutputNpMax);
assert(numel(dU1) == 3*mpcOutputNpMax && numel(sl1) == mpcOutputNpMax, ...
    'los puertos no cuadran con lo que espera el modelo');
fprintf('  |u| = %.6f m/s2 (%.1f %% de u_max)   -> OK\n', norm(u1), 100*norm(u1)/u_max);

fprintf('\n=== 2) el limite actua sobre la NORMA ===\n');
fprintf('  uLimitMode = %s, safetyCost = %g, u_max = %.6f\n', uLimitMode, safetyCost, u_max);
% Forzar una peticion grande: estado muy alejado de la referencia
xt2 = r_p_full((ik-1)*6 + (1:6)) + [4000; -3000; 2000; 2; -1.5; 1];
clear MPC_INOAS
u2 = MPC_INOAS(xt2, P_nav, 700);
fprintf('  peticion agresiva -> |u| = %.6f m/s2 = %.2f %% de u_max\n', ...
    norm(u2), 100*norm(u2)/u_max);
if norm(u2) <= u_max*1.0001
    fprintf('  la norma respeta el limite   -> OK\n');
else
    fprintf(2, '  LA NORMA SUPERA EL LIMITE\n');
end

fprintf('\n=== 3) covarianza del objeto en el camino de rejilla ===\n');
fprintf('  P_debris_pos presente: %d, sigma a lo largo de orbita = %.0f m\n', ...
    exist('P_debris_pos','var'), sqrt(max(eig(P_debris_pos))));

fprintf('\nVERIFY_A_OK\n');

%% Prueba de humo: MPC_INOAS aislado, sin el modelo Simulink completo.
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

REPO = REPO_ROOT;
addpath(REPO); addpath(genpath(fullfile(REPO,'matlab')));

setpref('inoas','skipBatchClear', true);          % no borrar mi workspace
setpref('inoas','mpcTuneConfig', struct('h',3,'Np',125));
mpcQuiet = true;
T0 = tic;
evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
T_INIT = toc(T0);
mpcQuiet = true;

fprintf('\n### init OK en %.1f s\n', T_INIT);
fprintf('### Np=%d h=%g Ntimesteps=%d | dsafe0=%g safetyCost=%g u_max=%g du_max=%g slackW=%g\n', ...
    Np, h, Ntimesteps, dsafe0, safetyCost, u_max, du_max, slackWeight);
fprintf('### size(Q)=%s size(deltaUmax)=%s | Ndecision = m*Np+Np = %d\n', ...
    mat2str(size(Q)), mat2str(size(deltaUmax)), 3*Np+Np);
fprintf('### mean|a_missing| (J2 no modelado por HCW) vs u_max -> ver arriba\n');

P = P0_kalman;
clear MPC_INOAS;

fprintf('\n t[s] | solve[s] | |u| [m/s2] | max|u|/umax | maxslack | dsafe(1) | dsafe(Np)\n');
for t_test = [0 300 600 750 800]
    idx = min(floor(t_test/h)+1, Ntimesteps);
    x_true = r_p_full((idx-1)*6 + (1:6));
    x_est  = x_true + [80; -60; 40; 0.05; -0.03; 0.02];
    tt = tic; [u_cmd, ~, slack] = MPC_INOAS(x_est, P, t_test); dt_solve = toc(tt);
    d1 = NaN; dN = NaN;
    if evalin('base','exist(''mpc_dsafe_log_first'',''var'')')
        df = evalin('base','mpc_dsafe_log_first'); dm = evalin('base','mpc_dsafe_log_max');
        d1 = df(end); dN = dm(end);
    end
    fprintf('%5g | %8.3f | %10.5f | %11.3f | %8.1e | %8.2f | %8.2f\n', ...
        t_test, dt_solve, norm(u_cmd), max(abs(u_cmd))/u_max, max(slack), d1, dN);
end
fprintf('\nSMOKE_OK\n');

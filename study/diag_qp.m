%DIAG_QP Donde falla el QP, y con que algoritmo.
% El 22 % de los solves agota iteraciones. Antes de seguir tocando el escalado
% hay que saber si los fallos se concentran en el cruce del debris (problema
% casi infactible) o estan repartidos (problema de condicionamiento puro).
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
addpath(STUDY_DIR);
P_nav = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);

for alg = ["interior-point-convex", "active-set"]
    setpref('inoas', 'skipBatchClear', true);
    setpref('inoas', 'mpcTuneConfig', struct('h', 3, 'Np', 125));
    mpcQuiet = true;
    evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
    mpcQuiet = true;
    mpcQpAlgorithm = alg;

    fprintf('\n===== %s  (h=3, Np=125, debris en t=800 s) =====\n', alg);
    fprintf('%8s %10s %8s %8s %12s %12s\n', ...
        't[s]', 'solve[s]', 'exitflag', 'iters', 'slack max', '|u|');

    clear MPC_INOAS
    tp = 500:25:900;
    for kp = 1:numel(tp)
        ikp = min(floor(tp(kp)/h)+1, Ntimesteps);
        xp = r_p_full((ikp-1)*6 + (1:6)) + [30; -20; 15; 0.02; -0.01; 0.01];
        uu = MPC_INOAS(xp, P_nav, tp(kp));
        d = evalin('base', 'mpc_diag_log');
        flag = '';
        if d.exitflag(end) <= 0; flag = '   <-- NO CONVERGE'; end
        fprintf('%8g %10.4f %8d %8g %12.3e %12.6f%s\n', ...
            tp(kp), d.solve_time(end), d.exitflag(end), d.iterations(end), ...
            d.slack_max(end), norm(uu), flag);
    end
    d = evalin('base', 'mpc_diag_log');
    fprintf('  resumen: %.1f %% convergidos | mediana %.4f s | peor %.4f s\n', ...
        100*mean(d.exitflag > 0), median(d.solve_time), max(d.solve_time));
end
fprintf('\nDIAG_QP_OK\n');

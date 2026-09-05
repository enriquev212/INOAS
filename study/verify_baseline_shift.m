%VERIFY_BASELINE_SHIFT Cuanto se mueve el numero publicado al corregir Q_cov.
% El abstract dice que la keep-out zone inflada llega a 270.72 m con la
% configuracion h = 3 s, Np = 125 (Th = 375 s). Hay que saber que valor da esa
% MISMA configuracion con el ruido de proceso bien escalado, porque es el numero
% que habra que reescribir.
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

fprintf('Configuracion publicada: h = 3 s, Np = 125, Th = 375 s\n\n');
for modo = ["legacy", "vanloan"]
    setpref('inoas', 'skipBatchClear', true);
    setpref('inoas', 'mpcTuneConfig', struct('h', 3, 'Np', 125));
    mpcQuiet = true;
    evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
    mpcQuiet = true;
    mpcVanLoanQ = strcmp(modo, "vanloan");

    clear MPC_INOAS mpc_dsafe_log_first mpc_dsafe_log_max
    ikk = min(floor(600/h)+1, Ntimesteps);
    xtt = r_p_full((ikk-1)*6 + (1:6));
    MPC_INOAS(xtt + [30; -20; 15; 0.02; -0.01; 0.01], P_nav, 600);
    fprintf('  %-8s -> dsafe paso 1 = %7.2f m | fin de horizonte = %7.2f m\n', ...
        modo, mpc_dsafe_log_first(end), mpc_dsafe_log_max(end));
end
fprintf('\n  (abstract publicado: 154 m de radio robusto, 270.72 m de maximo)\n');
fprintf('\nBASELINE_SHIFT_OK\n');

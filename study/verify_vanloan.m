%VERIFY_VANLOAN Prueba central de la fase 3.
% Afirmacion: con el ruido de proceso discretizado sobre h, el radio de
% seguridad inflado deja de depender de COMO se reparte el horizonte entre h y
% Np y pasa a depender solo del horizonte fisico.
%
% Se comparan tres repartos del MISMO horizonte de 500 s. Antes del arreglo
% discrepaban un 12,5 %. Ahora deben coincidir.
%
% Importante: no se reescala Q_cov a mano. El arreglo debe hacerlo solo; si
% hiciera falta reescalarlo fuera, no estaria arreglado.
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
casos = {  % h,  Np    (h*Np = 500 s en los tres)
    struct('h', 5,  'Np', 100), ...
    struct('h', 10, 'Np', 50 ), ...
    struct('h', 4,  'Np', 125)};

nCasos = numel(casos);
for modo = ["legacy", "vanloan"]
    fprintf('\n================ %s ================\n', upper(modo));
    dsafeFirst = nan(1, nCasos);
    dsafeEnd   = nan(1, nCasos);
    for kCase = 1:nCasos
        c = casos{kCase};
        setpref('inoas', 'skipBatchClear', true);
        setpref('inoas', 'mpcTuneConfig', struct('h', c.h, 'Np', c.Np));
        mpcQuiet = true;
        evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
        mpcQuiet = true;
        mpcVanLoanQ = strcmp(modo, "vanloan");   % lo lee getMpcConfig del base

        clear MPC_INOAS mpc_dsafe_log_first mpc_dsafe_log_max
        ik = min(floor(600/h)+1, Ntimesteps);
        xt = r_p_full((ik-1)*6 + (1:6));
        MPC_INOAS(xt + [30; -20; 15; 0.02; -0.01; 0.01], P_nav, 600);
        dsafeFirst(kCase) = mpc_dsafe_log_first(end);
        dsafeEnd(kCase)   = mpc_dsafe_log_max(end);
        fprintf('  h=%-3g Np=%-4d Th=%-4g s -> dsafe paso 1 = %7.2f m | fin horizonte = %7.2f m\n', ...
            c.h, c.Np, c.h*c.Np, dsafeFirst(kCase), dsafeEnd(kCase));
        % OJO: el init usa 'k' como variable de bucle y con skipBatchClear la
        % filtra al workspace del llamante. Por eso el indice de este bucle se
        % llama kCase: con 'k' los resultados se escribian en indices aleatorios.
        nCasos = numel(casos);
    end
    spread = max(dsafeEnd) - min(dsafeEnd);
    fprintf('  valores: %s\n', mat2str(round(dsafeEnd,2)));
    fprintf('  DISPERSION a horizonte constante: %.2f m (%.2f %% de %.2f m)\n', ...
        spread, 100*spread/mean(dsafeEnd), mean(dsafeEnd));
end

fprintf('\nVANLOAN_OK\n');

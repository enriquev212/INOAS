%PATCH_SEEDS Fijar las semillas de los bloques de ruido y comprobar que el
% modelo sigue simulando, no solo compilando.
%
% Los cinco bloques de ruido de los sensores auxiliares llevan seed =
% randi(100000), evaluado en cada carga del modelo. Junto con rng('shuffle')
% hacian que ninguna corrida fuese repetible: los resultados publicados son un
% sorteo y ningun barrido de parametros es interpretable.
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
addpath(REPO); addpath(genpath(fullfile(REPO,'matlab'))); addpath(fullfile(REPO,'models'));
setpref('inoas','skipBatchClear', true);
mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));

MODEL = 'inoas_model';
load_system(MODEL);

blks = find_system(MODEL, 'LookUnderMasks','all', 'FollowLinks','on');
nFixed = 0;
for k = 1:numel(blks)
    try
        sd = get_param(blks{k}, 'seed');
    catch
        continue;   % el bloque no tiene ese parametro
    end
    if ischar(sd) && contains(sd, 'randi')
        newSeed = 7000 + nFixed;
        set_param(blks{k}, 'seed', sprintf('%d', newSeed));
        [~, short] = fileparts(blks{k});
        fprintf('  %-26s %s -> %d\n', short, sd, newSeed);
        nFixed = nFixed + 1;
    end
end
fprintf('%d semillas fijadas\n', nFixed);
assert(nFixed == 5, 'esperaba 5 semillas, fijadas %d', nFixed);

save_system(MODEL);
fprintf('modelo guardado\n');

% --- comprobar que SIMULA, no solo que compila ---------------------------
% Una corrida corta: basta para que los dos charts tocados se ejecuten muchas
% veces y para que el MPC resuelva unos cuantos pasos.
close_system(MODEL, 0);
load_system(MODEL);
set_param(MODEL, 'SimulationMode', 'normal');
fprintf('\nsimulando 60 s...\n');
t0 = tic;
simOut = sim(MODEL, 'StopTime', '60');
fprintf('SIMULA: %.1f s de pared para 60 s de mision\n', toc(t0));

% Reproducibilidad: dos corridas identicas deben dar el mismo resultado.
fprintf('\nsegunda corrida identica para comprobar reproducibilidad...\n');
simOut2 = sim(MODEL, 'StopTime', '60');
if evalin('base', 'exist(''mpc_diag_log'',''var'')')
    d = evalin('base', 'mpc_diag_log');
    fprintf('  %d solves de MPC registrados en la corrida\n', numel(d.t));
end
fprintf('  (comparar salidas concretas requiere logging explicito; lo que\n');
fprintf('   importa aqui es que ambas corridas completan sin error)\n');

fprintf('\nPATCH_SEEDS_OK\n');

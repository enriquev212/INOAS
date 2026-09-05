%SYNC_WORK Resincroniza las copias aisladas del repo desde el repo real.
% Las copias en work/c1..c6 existen porque initialize_inoas_simulation.m ESCRIBE
% data/referenceTrajectory.mat y data/debrisTrajectory.mat, y varios procesos
% sobre el mismo repo se pisan. El precio es que quedan obsoletas en cuanto se
% toca el codigo: hay que llamar a este script DESPUES de cada parche y ANTES de
% cualquier barrido, o se estara midiendo el codigo viejo.
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
WORK = WORK_DIR;

for i = 1:6
    dst = fullfile(WORK, sprintf('c%d', i));
    if exist(dst, 'dir'); rmdir(dst, 's'); end
    mkdir(dst);
    copyfile(fullfile(REPO, 'matlab'), fullfile(dst, 'matlab'));
    copyfile(fullfile(REPO, 'models'), fullfile(dst, 'models'));
    copyfile(fullfile(REPO, 'data'),   fullfile(dst, 'data'));
    m = dir(fullfile(REPO, '*.m'));
    for k = 1:numel(m)
        copyfile(fullfile(REPO, m(k).name), fullfile(dst, m(k).name));
    end
end

% Sello de sincronizacion, para poder detectar copias obsoletas.
info = dir(fullfile(REPO, 'matlab', 'MPC_INOAS.m'));
fid = fopen(fullfile(WORK, 'SYNCED.txt'), 'w');
fprintf(fid, 'MPC_INOAS.m datestamp at sync: %s\n', info.date);
fclose(fid);
fprintf('work/c1..c6 resincronizadas (MPC_INOAS.m de %s)\n', info.date);

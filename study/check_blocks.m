
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

addpath(REPO_ROOT); addpath(fullfile(REPO_ROOT,'models'));
load_system('inoas_model');
for nm = {'Compute J','Sum3','ToWS_trace_x','ToWS_trace_y','ToWS_trace_z'}
    b = ['inoas_model/' nm{1}];
    try
        fprintf('%-14s Commented=%-6s BlockType=%s\n', nm{1}, ...
            get_param(b,'Commented'), get_param(b,'BlockType'));
    catch ME
        fprintf('%-14s ERROR: %s\n', nm{1}, ME.message);
    end
end

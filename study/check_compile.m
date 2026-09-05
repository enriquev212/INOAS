
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
evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
fprintf('workspace base poblado (Np=%d h=%g)\n', Np, h);
load_system('inoas_model');
fprintf('SimulationMode declarado en el modelo: %s\n', get_param('inoas_model','SimulationMode'));
set_param('inoas_model','SimulationMode','normal');
try
    set_param('inoas_model','SimulationCommand','update');   % compilacion de actualizacion
    fprintf('>>> EL MODELO ACTUALIZA/COMPILA CORRECTAMENTE <<<\n');
catch ME
    fprintf('>>> EL MODELO NO COMPILA <<<\n');
    fprintf('ID      : %s\n', ME.identifier);
    fprintf('MENSAJE : %s\n', ME.message);
    if isprop(ME,'cause')
        for c = 1:numel(ME.cause)
            fprintf('  causa %d: %s\n', c, ME.cause{c}.message);
        end
    end
end
% Censo independiente de puertos de entrada sin conectar en TODO el modelo
fprintf('\n--- puertos de entrada sin conectar ---\n');
pl = find_system('inoas_model','FindAll','on','LookUnderMasks','all','type','port','PortType','inport');
nbad = 0;
for k = 1:numel(pl)
    ln = get_param(pl(k),'Line');
    if ln <= 0
        blk = get_param(get_param(pl(k),'Parent'),'Name');
        fprintf('  SIN CONECTAR: bloque "%s", puerto %s\n', blk, get_param(pl(k),'PortNumber'));
        nbad = nbad + 1;
    end
end
fprintf('TOTAL puertos de entrada sin conectar: %d de %d\n', nbad, numel(pl));

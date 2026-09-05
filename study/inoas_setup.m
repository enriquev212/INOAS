%INOAS_SETUP Inicializa el workspace base para una configuracion (h, Np).
% SCRIPT, no funcion: MPC_INOAS lee su configuracion del base workspace con
% evalin, asi que todo tiene que vivir alli.
% Entrada:  CFG_ID (1..6)
% Salida:   workspace base poblado + STUDY_CFG con la definicion del caso.

% Copia aislada del repo por configuracion si existe: el init ESCRIBE
% data/referenceTrajectory.mat y data/debrisTrajectory.mat, asi que varios
% procesos sobre el mismo repo se pisan entre si.
% Poner STUDY_FORCE_MAIN = true para trabajar contra el repo real (proceso
% unico). Las copias quedan obsoletas en cuanto se parchea el codigo: si se usan
% sin resincronizar con sync_work, se mide el codigo viejo.
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

STUDY_WORK = fullfile(WORK_DIR, sprintf('c%d', CFG_ID));
if exist('STUDY_FORCE_MAIN', 'var') && STUDY_FORCE_MAIN
    STUDY_REPO = REPO_ROOT;
elseif exist(STUDY_WORK, 'dir')
    STUDY_REPO = STUDY_WORK;
else
    STUDY_REPO = REPO_ROOT;
end
STUDY_OUT  = OUT_DIR;
if ~exist(STUDY_OUT,'dir'); mkdir(STUDY_OUT); end
addpath(STUDY_REPO); addpath(genpath(fullfile(STUDY_REPO,'matlab')));
addpath(STUDY_DIR);

% Ya no hace falta reescalar du_max ni Q_cov a mano: el codigo lo hace solo.
% du_max se deriva de du_rate_max*h y Q_cov se rediscretiza sobre h con van
% Loan. Basta con dar el par (h, Np).
STUDY_C = { ...
  struct('name','h3_Np125_Th375',  'h',3,  'Np',125), ...  % linea base publicada
  struct('name','h5_Np100_Th500',  'h',5,  'Np',100), ...
  struct('name','h10_Np50_Th500',  'h',10, 'Np',50 ), ...
  struct('name','h5_Np150_Th750',  'h',5,  'Np',150), ...  % recomendada
  struct('name','h10_Np75_Th750',  'h',10, 'Np',75 ), ...  % alternativa barata
  struct('name','h5_Np200_Th1000', 'h',5,  'Np',200), ...  % limite superior
};
STUDY_CFG = STUDY_C{CFG_ID};

STUDY_TUNE = struct('h',STUDY_CFG.h,'Np',STUDY_CFG.Np);
setpref('inoas','skipBatchClear', true);
setpref('inoas','mpcTuneConfig', STUDY_TUNE);
mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", STUDY_REPO));
mpcQuiet = true;
rng(42);   % el init llama a rng('shuffle'); sin semilla fija nada es comparable

% P representativa de navegacion propagada (1-sigma ~ 12 m pos, 0.05 m/s vel).
% Elegida porque reproduce el radio robusto de 154 m que reporta el paper.
STUDY_PNAV = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);

fprintf('CONFIG %d: %s | h=%g Np=%d Th=%g s\n', ...
    CFG_ID, STUDY_CFG.name, h, Np, h*Np);
fprintf('  du_max=%.5f  Q_cov(4,4)=%.5f  slackWeight=%g  u_max=%g  nvars=%d\n', ...
    du_max, Q_cov_mpc(4,4), slackWeight, u_max, 3*Np+Np);

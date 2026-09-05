%RUN_NAVIGATION_CHAIN Mitad de navegacion de la cadena, medida en el modelo.
%
% La tesis del paper es una cadena causal: el ciclo de trabajo del GNSS degrada
% la navegacion, y esa degradacion encarece la maniobra de evasion. Hasta ahora
% las dos mitades estaban medidas por separado y el eslabon entre ellas era una
% hipotesis: el guiado recibia sigma como PARAMETRO.
%
% Este script mide la primera mitad sobre el modelo completo:
%   - el porcentaje de tiempo con GNSS activo (el 18,23 % del abstract, que hay
%     que volver a medir porque el contador del ciclo de trabajo estaba mal)
%   - la incertidumbre de navegacion frente al tiempo transcurrido desde la
%     ultima correccion, que es la cantidad transferible: con ella se predice
%     sigma en el TCA para cualquier antelacion y cualquier plan de ciclo.
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
addpath(STUDY_DIR);
OUT = OUT_DIR;

if ~exist('NAV_TF','var'); NAV_TF = 4000; end

setpref('inoas','skipBatchClear', true);
setpref('inoas','mpcTuneConfig', struct('h',3,'Np',125));
simulationStopTime = NAV_TF;
mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));
mpcQuiet = true;

MODEL = 'inoas_model';
load_system(MODEL);
set_param(MODEL, 'SimulationMode', 'normal');
set_param(MODEL, 'StopTime', num2str(NAV_TF));
set_param(MODEL, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

% Marcar la salida del decisor de instrumento para que quede registrada.
blk = [MODEL '/Instrument Decision'];
ph  = get_param(blk, 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on');
set_param(ph.Outport(1), 'DataLoggingNameMode', 'Custom');
set_param(ph.Outport(1), 'DataLoggingName', 'lambda_log');
fprintf('senal lambda marcada para logging\n');

clear MPC_INOAS mpc_diag_log
fprintf('simulando %d s...\n', NAV_TF);
t0 = tic;
so = sim(MODEL, 'StopTime', num2str(NAV_TF));
fprintf('  %.1f s de pared\n', toc(t0));

%% ---- ciclo de trabajo del GNSS ----------------------------------------
lam = so.logsout.get('lambda_log').Values;
tl  = lam.Time(:);  vl = double(lam.Data(:) > 0.5);
% Fraccion de TIEMPO, no de muestras: los pasos no son uniformes
dt  = diff([tl; tl(end)]);
onFrac = sum(dt .* vl) / sum(dt);
fprintf('\n--- Ciclo de trabajo del GNSS ---\n');
fprintf('  tiempo con GNSS activo: %.2f %%  (publicado: 18,23 %%)\n', 100*onFrac);

% Tiempo transcurrido desde la ultima correccion, en cada instante
gap = zeros(size(tl));
last_on = tl(1);
for k = 1:numel(tl)
    if vl(k) > 0.5; last_on = tl(k); end
    gap(k) = tl(k) - last_on;
end
fprintf('  hueco de propagacion: mediana %.0f s, maximo %.0f s\n', median(gap), max(gap));

%% ---- incertidumbre de navegacion frente al hueco ----------------------
D = evalin('base', 'mpc_diag_log');
sg = interp1(tl, gap, D.t, 'previous', 'extrap');
fprintf('\n--- Incertidumbre de navegacion ---\n');
fprintf('  sigma al recibir correccion (hueco < 5 s) : %.1f m\n', ...
    median(D.sigma_nav(sg < 5)));
fprintf('  sigma con hueco > 200 s                   : %.1f m\n', ...
    median(D.sigma_nav(sg > 200)));
fprintf('  sigma maxima observada                    : %.1f m\n', max(D.sigma_nav));

% Ley sigma(hueco): es lo que la capa de navegacion entrega al guiado
edges = 0:30:max(300, max(sg));
law = nan(1, numel(edges)-1); cnt = zeros(1, numel(edges)-1);
for k = 1:numel(edges)-1
    sel = sg >= edges(k) & sg < edges(k+1);
    cnt(k) = sum(sel);
    if any(sel); law(k) = median(D.sigma_nav(sel)); end
end
fprintf('\n  %10s %12s %8s\n', 'hueco [s]', 'sigma [m]', 'n');
for k = 1:numel(law)
    if cnt(k) > 0
        fprintf('  %4.0f - %-4.0f %11.1f %8d\n', edges(k), edges(k+1), law(k), cnt(k));
    end
end

NAV = struct('t', D.t, 'sigma', D.sigma_nav, 'gap', sg, ...
             'onFrac', onFrac, 'edges', edges, 'law', law, 'cnt', cnt, ...
             't_lambda', tl, 'lambda', vl, 'tf', NAV_TF);
save(fullfile(OUT,'nav_chain.mat'), 'NAV');
% Descartar el marcado de logging: no debe quedarse en el modelo versionado.
close_system(MODEL, 0);
fprintf('\nNAV_CHAIN_OK\n');

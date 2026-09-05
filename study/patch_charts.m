%PATCH_CHARTS Fase 6: quitar las constantes de tasa escondidas del modelo.
% Toca el .slx, asi que verifica despues de cada paso y aborta si algo no cuadra.
%
% 1) gatillo_gnss: gnss_rate = 3 literal, y mod(round(t),3) que con Ts = 0.5 s
%    dispara DOS veces por epoca (round(2.5) y round(3.0) valen ambos 3), la
%    mitad de ellas antes del flanco del ZOH.
% 2) instrument_decision: Ts = 1 literal y timer_count += int32(Ts), o sea un
%    contador de EJECUCIONES disfrazado de segundos. Con Ts = 0.5, int32(0.5)
%    vale 1 y las ventanas del duty cycle se parten por la mitad.
% 3) semillas aleatorias de los bloques de ruido.

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
fprintf('modelo cargado\n');

rt = sfroot;

%% ---- 1) gatillo del GNSS ------------------------------------------------
% Los charts se llaman "MATLAB Function"; la funcion va dentro del script.
allCharts = rt.find('-isa','Stateflow.EMChart');
ch = findChartByScript(allCharts, 'function enable_gnss = gatillo_gnss');
assert(numel(ch) == 1, 'esperaba 1 chart gatillo_gnss, encontrados %d', numel(ch));
old1 = ch.Script;
assert(contains(old1, 'gnss_rate = 3'), 'el chart no contiene gnss_rate = 3');

new1 = strjoin({
'function enable_gnss = gatillo_gnss(lambda, t)'
'    %#codegen'
'    % Opens the UKF measurement update only when a fresh GNSS epoch is'
'    % available AND the decision logic asks for it.'
'    %'
'    % The cadence comes from inoas_gnss_epoch(), the same source of truth as'
'    % gnss_sample_time, instead of a local literal kept in sync by hand.'
'    %'
'    % The previous test was mod(round(t), gnss_rate) == 0, which fires twice'
'    % per epoch as soon as the block runs faster than 1 Hz: at Ts = 0.5 s both'
'    % round(2.5) and round(3.0) give 3, and the earlier of the two lands BEFORE'
'    % the zero-order hold edge, so the filter would correct with a stale'
'    % position while weighting it with the fresh-measurement covariance.'
'    % Detecting the epoch change instead fires exactly once per epoch at any'
'    % execution rate that divides it.'
'    gnss_rate = inoas_gnss_epoch();'
''
'    persistent last_epoch'
'    if isempty(last_epoch)'
'        last_epoch = int32(-1);'
'    end'
''
'    epoch = int32(floor(t / gnss_rate + 1e-9));'
'    is_new_data = (epoch ~= last_epoch);'
'    if is_new_data'
'        last_epoch = epoch;'
'    end'
''
'    enable_gnss = lambda && is_new_data;'
'end'
}, newline);
ch.Script = new1;
fprintf('1) gatillo_gnss reescrito (%d -> %d caracteres)\n', numel(old1), numel(new1));

%% ---- 2) temporizador del duty cycle -------------------------------------
ch2 = findChartByScript(allCharts, 'function lambda = instrument_decision');
assert(numel(ch2) == 1, 'esperaba 1 chart instrument_decision, encontrados %d', numel(ch2));
old2 = ch2.Script;
assert(contains(old2, 'Ts          = 1;'), 'el chart no contiene el Ts literal esperado');
assert(contains(old2, 'timer_count = timer_count + int32(Ts);'), 'no encuentro el acumulador');

new2 = old2;
new2 = strrep(new2, ...
    'Ts          = 1;    % [s] execution sample time of this decision block', ...
    ['Ts          = inoas_estimator_dt();  % [s] execution sample time' newline ...
     '% This block used to carry its own literal Ts = 1 and accumulate' newline ...
     '% timer_count + int32(Ts), i.e. it counted EXECUTIONS while presenting them' newline ...
     '% as seconds. At Ts = 0.5 s the accumulator was doubly wrong: int32(0.5)' newline ...
     '% rounds to 1, so T_gnss_on = 60 meant 30 real seconds and the published' newline ...
     '% GNSS on-time moved by a counter artefact. Count executions explicitly and' newline ...
     '% convert the thresholds instead, which is exact at any rate.' newline ...
     'n_gnss_on   = int32(round(T_gnss_on   / Ts));' newline ...
     'n_kalman_on = int32(round(T_kalman_on / Ts));']);
new2 = strrep(new2, ...
    'timer_count = timer_count + int32(Ts);', ...
    'timer_count = timer_count + int32(1);   % executions, not seconds');
new2 = strrep(new2, 'timer_count >= int32(T_gnss_on)',   'timer_count >= n_gnss_on');
new2 = strrep(new2, 'timer_count >= int32(T_kalman_on)', 'timer_count >= n_kalman_on');

assert(~contains(new2, 'int32(T_gnss_on)'),   'quedo un umbral sin convertir (gnss)');
assert(~contains(new2, 'int32(T_kalman_on)'), 'quedo un umbral sin convertir (kalman)');
ch2.Script = new2;
fprintf('2) instrument_decision reescrito (%d -> %d caracteres)\n', numel(old2), numel(new2));

%% ---- 3) semillas de los bloques de ruido --------------------------------
blks = find_system(MODEL, 'LookUnderMasks','all', 'FollowLinks','on', ...
    'RegExp','on', 'BlockType','RandomNumber');
blks = [blks; find_system(MODEL, 'LookUnderMasks','all', 'FollowLinks','on', ...
    'RegExp','on', 'BlockType','UniformRandomNumber')];
nSeed = 0;
for k = 1:numel(blks)
    s = get_param(blks{k}, 'Seed');
    if contains(s, 'randi')
        set_param(blks{k}, 'Seed', sprintf('%d', 1000 + k));
        fprintf('   semilla fija en %s: %s -> %d\n', blks{k}, s, 1000 + k);
        nSeed = nSeed + 1;
    end
end
fprintf('3) %d semillas aleatorias fijadas (de %d bloques de ruido)\n', nSeed, numel(blks));

%% ---- guardar y verificar ------------------------------------------------
save_system(MODEL);
fprintf('\nmodelo guardado\n');

close_system(MODEL, 0);
load_system(MODEL);
rt = sfroot;
allCharts = rt.find('-isa','Stateflow.EMChart');
c1 = findChartByScript(allCharts, 'function enable_gnss = gatillo_gnss');
c2 = findChartByScript(allCharts, 'function lambda = instrument_decision');
assert(contains(c1.Script, 'inoas_gnss_epoch()'), 'el gatillo no persistio');
assert(contains(c2.Script, 'inoas_estimator_dt()'), 'el decisor no persistio');
fprintf('releido: los dos charts conservan los cambios\n');

set_param(MODEL, 'SimulationMode', 'normal');
try
    set_param(MODEL, 'SimulationCommand', 'update');
    fprintf('EL MODELO SIGUE COMPILANDO\n');
catch ME
    fprintf(2, 'EL MODELO NO COMPILA: %s\n', ME.message);
    error('patch_charts:compile', 'abortado');
end
fprintf('\nPATCH_CHARTS_OK\n');


function out = findChartByScript(charts, needle)
%FINDCHARTBYSCRIPT Los EMChart se llaman "MATLAB Function": buscar por contenido.
    out = [];
    for i = 1:numel(charts)
        if contains(charts(i).Script, needle)
            out = [out; charts(i)]; %#ok<AGROW>
        end
    end
end

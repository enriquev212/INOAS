%BENCH_CONSECUTIVE Coste por solve en operacion realista.
% Resuelve en pasos CONSECUTIVOS de h segundos, que es como corre el
% controlador: el warm start del paso anterior sigue siendo valido y el conjunto
% activo apenas cambia. Sondear cada 25 s, como hacia bench_solver, deja el warm
% start obsoleto y sobreestima el coste.
%
% BENCH_REPO permite apuntar a otra copia del repositorio (un worktree de git)
% para comparar contra una version anterior con el mismo protocolo.
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

if ~exist('BENCH_REPO', 'var')
    BENCH_REPO = REPO_ROOT;
end
if ~exist('BENCH_TAG', 'var'); BENCH_TAG = 'actual'; end

addpath(BENCH_REPO); addpath(genpath(fullfile(BENCH_REPO, 'matlab')));
P_nav = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);
casos = { struct('h',3,'Np',125), struct('h',5,'Np',100), ...
          struct('h',10,'Np',50), struct('h',5,'Np',150) };
nCasos = numel(casos);
T0 = 500; T1 = 900;          % cubre el encuentro en t = 800 s

fprintf('=== %s (%s) ===\n', BENCH_TAG, BENCH_REPO);
fprintf('%-20s %7s %8s %9s %9s %9s %8s %8s\n', ...
    'configuracion', 'solves', 'mediana', 'p90', 'p99', 'peor', 'plazo', '%% ok');
fprintf('%s\n', repmat('-', 1, 86));

filas = cell(1, nCasos);
for kCase = 1:nCasos
    c = casos{kCase};
    setpref('inoas', 'skipBatchClear', true);
    setpref('inoas', 'mpcTuneConfig', struct('h', c.h, 'Np', c.Np));
    mpcQuiet = true;
    evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", BENCH_REPO));
    mpcQuiet = true;

    clear MPC_INOAS
    tps = T0:c.h:T1;
    ts = zeros(1, numel(tps));
    okv = zeros(1, numel(tps));
    for kp = 1:numel(tps)
        ikp = min(floor(tps(kp)/h)+1, Ntimesteps);
        xp = r_p_full((ikp-1)*6 + (1:6)) + [30; -20; 15; 0.02; -0.01; 0.01];
        tt = tic; MPC_INOAS(xp, P_nav, tps(kp)); ts(kp) = toc(tt);
        nCasos = numel(casos);          % el init filtra variables al workspace
    end
    if evalin('base', 'exist(''mpc_diag_log'',''var'')')
        d = evalin('base', 'mpc_diag_log');
        okv = d.exitflag > 0;
    else
        okv = nan(size(ts));            % version antigua, sin instrumentacion
    end

    r = struct('tag', BENCH_TAG, 'h', c.h, 'Np', c.Np, 'n', numel(ts), ...
        'med', median(ts), 'p90', prctile(ts,90), 'p99', prctile(ts,99), ...
        'worst', max(ts), 'ok', 100*mean(okv), 'ts', ts);
    filas{kCase} = r;
    fprintf('h=%-3g Np=%-4d Th=%-5g %7d %8.4f %9.4f %9.4f %9.4f %8g %8.1f\n', ...
        c.h, c.Np, c.h*c.Np, r.n, r.med, r.p90, r.p99, r.worst, c.h, r.ok);
end

outf = fullfile(OUT_DIR, ...
    sprintf('bench_consec_%s.mat', BENCH_TAG));
save(outf, 'filas');
fprintf('\nBENCH_CONSEC_OK %s\n', BENCH_TAG);

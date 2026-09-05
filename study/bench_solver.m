%BENCH_SOLVER Coste por solve, antes y despues de la fase 4.
% Separa la PRIMERA llamada de una configuracion (que construye las matrices de
% prediccion) de las siguientes (que las reutilizan). Para un controlador
% embarcado el numero que importa es el peor caso en regimen, no la media.
%
% Secuencial a proposito: los tiempos solo son comparables sin contencion de CPU.
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
casos = { struct('h',3,'Np',125), struct('h',5,'Np',100), ...
          struct('h',10,'Np',50), struct('h',5,'Np',150) };
nCasos = numel(casos);
tWarm  = 500:25:900;      % 17 solves en regimen, cubriendo el encuentro

fprintf('%-22s %8s %8s %9s %9s %9s %8s %7s\n', ...
    'configuracion', 'vars', 'frio[s]', 'mediana', 'p90', 'peor', 'plazo', 'ratio');
fprintf('%s\n', repmat('-', 1, 92));

resumen = cell(1, nCasos);
for kCase = 1:nCasos
    c = casos{kCase};
    setpref('inoas', 'skipBatchClear', true);
    setpref('inoas', 'mpcTuneConfig', struct('h', c.h, 'Np', c.Np));
    mpcQuiet = true;
    evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
    mpcQuiet = true;

    % --- llamada en frio: incluye construir Phi, Gamma_u, Gamma_extend ---
    clear MPC_INOAS
    ikc = min(floor(400/h)+1, Ntimesteps);
    xc  = r_p_full((ikc-1)*6 + (1:6)) + [30; -20; 15; 0.02; -0.01; 0.01];
    tc = tic; MPC_INOAS(xc, P_nav, 400); tCold = toc(tc);

    % --- llamadas en regimen: cache caliente, warm start activo ----------
    nW = numel(tWarm);
    tw = zeros(1, nW);
    for kw = 1:nW
        ikw = min(floor(tWarm(kw)/h)+1, Ntimesteps);
        xw  = r_p_full((ikw-1)*6 + (1:6)) + [30; -20; 15; 0.02; -0.01; 0.01];
        t0 = tic; MPC_INOAS(xw, P_nav, tWarm(kw)); tw(kw) = toc(t0);
        nCasos = numel(casos);   % el init filtra variables; releer
    end

    d = evalin('base', 'mpc_diag_log');
    r = struct('h', c.h, 'Np', c.Np, 'nvars', 4*c.Np, 'cold', tCold, ...
        'med', median(tw), 'p90', prctile(tw, 90), 'worst', max(tw), ...
        'ratio', max(tw)/c.h, 'iters_med', median(d.iterations), ...
        'iters_max', max(d.iterations), 'okfrac', mean(d.exitflag > 0));
    resumen{kCase} = r;

    fprintf('h=%-3g Np=%-4d Th=%-5g %8d %8.3f %9.4f %9.4f %9.4f %8g %7.3f\n', ...
        c.h, c.Np, c.h*c.Np, 4*c.Np, tCold, r.med, r.p90, r.worst, c.h, r.ratio);
end

fprintf('\n%-22s %10s %10s %8s\n', 'configuracion', 'iter med', 'iter max', '%% ok');
for kCase = 1:numel(resumen)
    r = resumen{kCase};
    fprintf('h=%-3g Np=%-4d %14g %10g %8.1f\n', ...
        r.h, r.Np, r.iters_med, r.iters_max, 100*r.okfrac);
end

save(fullfile(OUT_DIR,'bench_solver.mat'), 'resumen');
fprintf('\nBENCH_OK\n');

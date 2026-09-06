%MAKE_ARCH_FIGURE La figura que justifica la arquitectura.
%  ============================================================================
%  SUPERSEDIDO. No genera las figuras del articulo.
%
%  Las figuras del paper se dibujan ahora en study/pyfigs (Python), a partir de
%  los CSV que exporta study/pyfigs/export_data.py. Este script se conserva por
%  historia y porque algunos de sus bloques todavia producen .mat de campana,
%  pero sus FIGURAS estan obsoletas y algunas de sus fuentes tambien: carga
%  bench_consec_quadprog_final.mat, medido con codigo anterior a 8d55826.
%
%  Para regenerar las figuras:   cd study/pyfigs && python make_all.py
%  ============================================================================
%
% Contrasta el coste por solve de las dos formas de imponer la evitacion:
%   (a) restriccion dentro del MPC, muestreada en la rejilla de prediccion
%   (b) guiado planifica la maniobra y el MPC solo sigue
% Con un margen de seguridad defendible (3 sigma) la primera queda cerca de la
% frontera de factibilidad, y eso se paga en tiempo de solve y en convergencia.
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

addpath(STUDY_DIR);
OUT  = FIG_DIR;
DATA = OUT_DIR;

G = load(fullfile(DATA,'bench_consec_quadprog_final.mat'));   % ruta de rejilla
P = load(fullfile(DATA,'primary_timing.mat'));                % ruta primaria

W = 3.5; H = 2.6;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',8);
set(groot,'defaultLineLineWidth',1.0);
Cr=[.84 .19 .15]; Cg=[0 .55 .40]; Ck=[.45 .45 .45];

fg = figure('Visible','off');

% Ruta de rejilla: una columna por configuracion
labs = {}; k = 0;
for i = 1:numel(G.filas)
    f = G.filas{i};
    k = k + 1;
    ts = f.ts;
    plot(k*ones(size(ts)), ts, '.', 'Color', [Cr 0.30], 'MarkerSize', 4); hold on
    plot(k, median(ts), '_', 'Color', Cr, 'MarkerSize', 16, 'LineWidth', 1.6);
    plot([k-0.3 k+0.3], f.h*[1 1], 'k--');
    labs{k} = sprintf('h=%g, N_p=%d', f.h, f.Np); %#ok<SAGROW>
end

% Ruta primaria
k = k + 1;
ts = P.Dg.solve_time;
plot(k*ones(size(ts)), ts, '.', 'Color', [Cg 0.30], 'MarkerSize', 4);
plot(k, median(ts), '_', 'Color', Cg, 'MarkerSize', 16, 'LineWidth', 1.6);
plot([k-0.3 k+0.3], 30*[1 1], 'k--');
labs{k} = 'guiado + seguim.';

set(gca, 'YScale','log', 'XTick', 1:k, 'XTickLabel', labs, ...
    'XTickLabelRotation', 22, 'TickLabelInterpreter','tex');
ylabel('Tiempo de solve  [s]');
ylim([2e-3 3e2]); xlim([0.5 k+0.5]);
text(0.55, 1.3e2, 'discontinua: plazo de tiempo real', 'FontSize', 6.5);
legend({'restriccion en la rejilla','', '', 'guiado planifica, MPC sigue'}, ...
    'Location','southwest', 'FontSize', 6.5, 'Box','off');
grid on; set(gca,'GridAlpha',.12,'Layer','top');
savefig_ieee(fg, 'figI_architecture_cost', OUT, W, H);

fprintf('resumen numerico:\n');
for i = 1:numel(G.filas)
    f = G.filas{i};
    fprintf('  rejilla  h=%-3g Np=%-4d mediana %8.4f s  peor %9.4f s  ok %5.1f %%\n', ...
        f.h, f.Np, median(f.ts), max(f.ts), f.ok);
end
fprintf('  primaria (h=30)      mediana %8.4f s  peor %9.4f s  ok %5.1f %%\n', ...
    median(P.Dg.solve_time), max(P.Dg.solve_time), 100*mean(P.Dg.exitflag>0));
fprintf('\nARCH_FIG_OK\n');

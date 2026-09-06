%MAKE_PAPER_FIGURES Las seis figuras del paper, con acabado de publicacion.
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
% Un unico estilo (inoas_figstyle / inoas_axstyle), anotaciones colocadas a mano
% donde antes se pisaban entre ellas o pisaban los datos, y trazo suficiente para
% 3.5 in de ancho impreso.
STUDY_DIR = fileparts(mfilename('fullpath'));
REPO_ROOT = fileparts(STUDY_DIR);
OUT_DIR   = fullfile(STUDY_DIR, 'out');
FIG_DIR   = fullfile(STUDY_DIR, 'figures');
addpath(REPO_ROOT); addpath(genpath(fullfile(REPO_ROOT,'matlab'))); addpath(STUDY_DIR);

C = inoas_figstyle();
W = 3.5; H = 2.55;

%% ============ Fig 1: la superficie del acoplamiento =====================
S = load(fullfile(OUT_DIR,'mc_surface.mat'));
f = figure('Visible','off');
yl = [0 68];   % espacio para la leyenda por encima de la curva de 300 m
% La banda medida va primero, para que quede por detras de las curvas sin
% depender de uistack.
patch([4.8 32.1 32.1 4.8], [yl(1) yl(1) yl(2) yl(2)], C.lgry, ...
    'FaceAlpha', 0.13, 'EdgeColor','none'); hold on
set(gca,'XScale','log');
mk = {'o','s','^','d'};
hs = gobjects(1,numel(S.SO));
for i = 1:numel(S.SO)
    hs(i) = plot(S.SN, S.MED(i,:), '-', 'Color', C.order(i,:), ...
        'Marker', mk{i}, 'MarkerFaceColor', C.order(i,:), ...
        'MarkerEdgeColor','none', 'MarkerSize', 4);
end
% El hueco entre la curva de 100 m y la de 300 m esta vacio en todo el rango:
% ahi cabe la etiqueta de la banda sin tapar ningun dato.
text(12.4, 29, 'measured range', 'FontSize', 7, 'Color', C.gry, ...
    'HorizontalAlignment','center');
xlabel('Navigation uncertainty at the encounter  [m]');
ylabel('Median manoeuvre  \Deltav  [mm/s]');
xlim([4 300]); ylim(yl);
xticks([5 10 30 100 300]); xticklabels({'5','10','30','100','300'});
yticks(0:10:60);
legend(hs, arrayfun(@(x) sprintf('\\sigma_{object} = %d m', x), S.SO, ...
    'UniformOutput', false), 'Location','northwest');
inoas_axstyle(gca);
savefig_ieee(f, 'paper1_coupling_surface', FIG_DIR, W, H);

%% ============ Fig 2: la correccion de van Loan ==========================
P = load(fullfile(OUT_DIR,'koz_profiles.mat'));   % generado por make_figures
f = figure('Visible','off');
for kc = 1:3
    plot(P.tt{kc}, P.legacy{kc}, '-', 'Color', C.red, 'LineWidth', 1.0); hold on
end
mk2 = {'o','s','^'};
hv = gobjects(1,3);
for kc = 1:3
    idx = round(linspace(6, numel(P.tt{kc}), 6));
    idx = idx(1 + mod(kc-1,3) : 3 : end);
    hv(kc) = plot(P.tt{kc}, P.vanloan{kc}, '-', 'Color', C.grn, ...
        'Marker', mk2{kc}, 'MarkerIndices', idx, 'MarkerSize', 4.5, ...
        'MarkerFaceColor','w', 'MarkerEdgeColor', C.grn);
end
% Las tres rojas llegan a 260 m en el extremo derecho: la etiqueta va por debajo
% de ellas, en la esquina que queda vacia.
text(498, 186, sprintf('per-step\n(mesh dependent)'), 'FontSize', 7, ...
    'Color', C.red, 'HorizontalAlignment','right', 'VerticalAlignment','top');
text(215, 345, sprintf('time-consistent\n(three splittings, coincident)'), ...
    'FontSize', 7, 'Color', C.grn, 'HorizontalAlignment','center');
xlabel('Time into the prediction horizon  [s]');
ylabel('Inflated keep-out radius  [m]');
xlim([0 500]); ylim([150 400]);
inoas_axstyle(gca);
savefig_ieee(f, 'paper2_koz_consistency', FIG_DIR, W, H);

%% ============ Fig 3: coste de las dos arquitecturas =====================
G = load(fullfile(OUT_DIR,'bench_consec_quadprog_final.mat'));
Q = load(fullfile(OUT_DIR,'primary_timing.mat'));
f = figure('Visible','off');
labs = {};
for i = 1:numel(G.filas)
    fi = G.filas{i};
    jitter = (rand(size(fi.ts))-0.5)*0.24;
    plot(i+jitter, fi.ts, '.', 'Color', [C.red 0.25], 'MarkerSize', 3.5); hold on
    plot([i-0.30 i+0.30], median(fi.ts)*[1 1], '-', 'Color', C.red, 'LineWidth', 2);
    plot([i-0.36 i+0.36], fi.h*[1 1], '--', 'Color', [0.15 0.15 0.15], 'LineWidth', 0.8);
    labs{i} = sprintf('%g s / %d', fi.h, fi.Np); %#ok<SAGROW>
end
k = numel(G.filas)+1;
ts = Q.Dg.solve_time;
jitter = (rand(size(ts))-0.5)*0.24;
plot(k+jitter, ts, '.', 'Color', [C.grn 0.25], 'MarkerSize', 3.5);
plot([k-0.30 k+0.30], median(ts)*[1 1], '-', 'Color', C.grn, 'LineWidth', 2);
plot([k-0.36 k+0.36], 30*[1 1], '--', 'Color', [0.15 0.15 0.15], 'LineWidth', 0.8);
labs{k} = 'plan + track';
set(gca,'YScale','log','XTick',1:k,'XTickLabel',labs);
ylabel('Solve time  [s]');
ylim([1e-3 3e2]); xlim([0.4 k+0.6]);
yticks([1e-3 1e-2 1e-1 1e0 1e1 1e2]);
text(0.5, 1.4e2, 'dashed: real-time deadline', 'FontSize', 6.5, ...
    'Color', [0.15 0.15 0.15]);
text(2.5, 1.7e-3, 'constraint inside the QP:  step h / horizon N_p', ...
    'FontSize', 7, 'Color', C.red, 'HorizontalAlignment','center');
text(k, 0.45, sprintf('guidance\nplans'), 'FontSize', 7, 'Color', C.grn, ...
    'HorizontalAlignment','center');
% Sin etiqueta de eje x: describiria las cuatro primeras columnas pero no la
% quinta, y colocarla a mano desactiva el ajuste automatico de la caja.
inoas_axstyle(gca);
savefig_ieee(f, 'paper3_architecture_cost', FIG_DIR, W, H);

%% ============ Fig 4: ejecucion de la maniobra ===========================
% Dos paneles con eje x compartido. La version con yyaxis perdia la curva de
% aceleracion: al volver a 'yyaxis left' para pintar las verticales, plot con
% hold desactivado borra lo que ya habia en ese lado, incluida su etiqueta.
E = load(fullfile(OUT_DIR,'test_cam_retarget.mat'));
tmin = (E.tv - E.SC.t_tca)/60;
tb   = (E.PLAN.t_burn - E.SC.t_tca)/60;
f = figure('Visible','off');
tiledlayout(f, 2, 1, 'TileSpacing','compact', 'Padding','compact');

ax1 = nexttile;
plot(tmin, E.un*1e6, '-', 'Color', C.blue); hold on
plot([tb tb], [0 46], ':', 'Color', C.gry, 'LineWidth', 0.9);
plot([0 0],  [0 46], '-', 'Color', C.gry, 'LineWidth', 0.9);
text(tb+2.5, 45, 'manoeuvre', 'FontSize', 7, 'Color', C.gry, ...
    'VerticalAlignment','top');
text(-2.5, 45, 'encounter', 'FontSize', 7, 'Color', C.gry, ...
    'HorizontalAlignment','right', 'VerticalAlignment','top');
text(-56, 26, sprintf('%.1f mm/s planned, %.1f mm/s spent', ...
    abs(E.PLAN.dv)*1e3, E.dv_cum(end)*1e3), 'FontSize', 7, ...
    'Color', [0.15 0.15 0.15], 'HorizontalAlignment','center');
text(-56, 15, '0.1% of the 1 N authority', 'FontSize', 7, ...
    'Color', C.gry, 'HorizontalAlignment','center');
ylabel(ax1, sprintf('Commanded\nacceleration  [\\mum/s^2]'));
xlim([-120 15]); ylim([0 48]); yticks(0:10:40);
set(ax1, 'XTickLabel', []);
inoas_axstyle(ax1);

ax2 = nexttile;
plot(tmin, E.err, '-', 'Color', C.grn); hold on
plot([tb tb], [0 3], ':', 'Color', C.gry, 'LineWidth', 0.9);
plot([0 0],  [0 3], '-', 'Color', C.gry, 'LineWidth', 0.9);
text(-56, 2.1, sprintf('rms %.2f m', rms(E.err)), 'FontSize', 7, ...
    'Color', [0.15 0.15 0.15], 'HorizontalAlignment','center');
ylabel(ax2, sprintf('Tracking\nerror  [m]'));
xlabel(ax2, 'Time relative to the encounter  [min]');
xlim([-120 15]); ylim([0 3]); yticks(0:1:3);
inoas_axstyle(ax2);
savefig_ieee(f, 'paper4_execution', FIG_DIR, W, 3.1);

%% ============ Fig 5: la ley de navegacion medida ========================
N = load(fullfile(OUT_DIR,'nav_chain.mat')); NAV = N.NAV;
f = figure('Visible','off');
ctr = NAV.edges(1:end-1) + diff(NAV.edges)/2;
ok  = NAV.cnt > 0;
plot([0 300], [32.1 32.1], '--', 'Color', C.gry, 'LineWidth', 0.8); hold on
plot(ctr(ok), NAV.law(ok), '-', 'Color', C.blue, 'Marker','o', ...
    'MarkerFaceColor', C.blue, 'MarkerEdgeColor','none', 'MarkerSize', 4);
plot(ctr(1), NAV.law(1), 'o', 'Color', C.grn, 'MarkerFaceColor', C.grn, ...
    'MarkerEdgeColor','none', 'MarkerSize', 6);
text(294, 34.0, sprintf('saturates at %.1f m', 32.1), 'FontSize', 7, ...
    'Color', C.gry, 'HorizontalAlignment','right');
text(30, 7.4, sprintf('%.1f m after a fix', NAV.law(1)), 'FontSize', 7, ...
    'Color', C.grn);
xlabel('Time since the last GNSS fix  [s]');
ylabel('Navigation uncertainty  [m]');
xlim([0 300]); ylim([0 38]);
inoas_axstyle(gca);
savefig_ieee(f, 'paper5_navigation_law', FIG_DIR, W, H);

%% ============ Fig 6: efectividad del impulso tangencial =================
files = dir(fullfile(OUT_DIR,'mc_cam_0*.mat'));
A = [];
for k2 = 1:numel(files); Sm = load(fullfile(files(k2).folder,files(k2).name)); A = [A, Sm.M]; end %#ok<AGROW>
sgv = [A.sig]; first = sgv == min(sgv);
sov = [A.sig_obj]; first = first & sov == min(sov);
vr = [A(first).vrel]/1e3;  sn = [A(first).sens];

f = figure('Visible','off');
plot([2 15], [0 0], '-', 'Color', C.lgry, 'LineWidth', 0.8); hold on
scatter(vr, sn, 5, C.blue, 'filled', 'MarkerFaceAlpha', 0.25);
% Envolvente teorica: la ganancia va como cos(dInc/2), y v_rel = 2 v sin(dInc/2)
vv = linspace(2.5, 14.2, 200);
sth = 3*6743*sqrt(max(0, 1 - (vv/(2*7.188)).^2))/1e3;
plot(vv,  sth, '-', 'Color', C.red, 'LineWidth', 1.1);
plot(vv, -sth, '-', 'Color', C.red, 'LineWidth', 1.1);
text(4.3, 16.6, '\pm 3 T_{orb} cos(\Delta i/2)', 'FontSize', 7, 'Color', C.red);
text(13.7, 3.0, sprintf('head-on:\nblind'), 'FontSize', 7, ...
    'Color', [0.15 0.15 0.15], 'HorizontalAlignment','right');
xlabel('Relative velocity at the encounter  [km/s]');
ylabel('Miss opened per unit \Deltav  [m per mm/s]');
xlim([2 15]); ylim([-21 21]); yticks(-20:10:20);
inoas_axstyle(gca);
savefig_ieee(f, 'paper6_tangential_authority', FIG_DIR, W, H);

fprintf('PAPER_FIGURES_OK\n');

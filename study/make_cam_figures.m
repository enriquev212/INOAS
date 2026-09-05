%MAKE_CAM_FIGURES Figuras del demostrador de conjuncion realista.
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
S = load(fullfile(DATA,'cam_demo.mat'));

W = 3.5; H = 2.6; FS = 8;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',FS);
set(groot,'defaultLineLineWidth',1.0);
Cb = [0.00 0.45 0.70]; Cr = [0.84 0.19 0.15];
Cg = [0.00 0.55 0.40]; Cy = [0.90 0.55 0.10]; Ck = [0.45 0.45 0.45];

%% ===== Fig A: delta-v necesario frente a antelacion =====================
f = figure('Visible','off');
lead = [S.sweep.lead]/S.T_orb;  dv = [S.sweep.dv_req]*1e3;
semilogy(lead, dv, 'o-', 'Color', Cb, 'MarkerFaceColor', Cb, 'MarkerSize', 4);
hold on
yline(2500, '--', 'Color', Cr);
text(2.95, 3400, 'escenario co-orbital actual: 2500 mm/s', ...
    'FontSize', 6.5, 'Color', Cr, 'HorizontalAlignment','right');
xlabel('Antelacion de la maniobra respecto al TCA  [orbitas]');
ylabel('\Deltav necesario  [mm/s]');
ylim([5 6000]); xlim([0.3 3.3]);
grid on; set(gca,'GridAlpha',0.12,'Layer','top');
savefig_ieee(f, 'figA_cam_dv_vs_lead', OUT, W, H);

%% ===== Fig B: acoplamiento navegacion - combustible =====================
f = figure('Visible','off');
sg = [S.nav_sweep.sig];  dvn = [S.nav_sweep.dv]*1e3;
pc0 = [S.nav_sweep.Pc0]; pc1 = [S.nav_sweep.Pc1];
yyaxis left
plot(sg, dvn, 'o-', 'Color', Cb, 'MarkerFaceColor', Cb, 'MarkerSize', 4);
ylabel('\Deltav para margen 3\sigma  [mm/s]'); ylim([0 120]);
set(gca,'YColor',Cb);
yyaxis right
semilogy(sg, pc0, 's--', 'Color', Cr, 'MarkerFaceColor','w', 'MarkerSize', 4);
hold on
semilogy(sg, pc1, '^-', 'Color', Cg, 'MarkerFaceColor', Cg, 'MarkerSize', 4);
ylabel('Probabilidad de colision'); ylim([1e-6 3e-3]);
set(gca,'YColor',Cr);
xlabel('Incertidumbre de navegacion en el TCA, 1\sigma  [m]');
legend({'\Deltav requerido','P_c sin maniobra','P_c tras maniobra'}, ...
    'Location','northwest','FontSize',6.5,'Box','off');
grid on; set(gca,'GridAlpha',0.12,'Layer','top');
savefig_ieee(f, 'figB_cam_nav_coupling', OUT, W, H);

%% ===== Fig C: el plano-B, antes y despues ===============================
% La figura clasica de evitacion de colision: vector de fallo, elipse de
% covarianza combinada y radio de cuerpo duro, proyectados al plano-B.
f = figure('Visible','off');
sig = 40;  P_nav = diag([sig sig sig].^2);
B0 = cam_bplane(S.T0, P_nav, S.P_deb, S.R_HB);

th = linspace(0, 2*pi, 200);
[V, D] = eig(B0.C);  Lr = sqrt(max(diag(D),0));
ell = @(k) (V * [k*Lr(1)*cos(th); k*Lr(2)*sin(th)]);

for k = [1 3]
    E = ell(k);
    plot(E(1,:), E(2,:), '-', 'Color', [Cr 0.85], 'LineWidth', 0.8); hold on
    text(0, k*Lr(2)*1.02, sprintf('%d\\sigma', k), 'FontSize', 6, ...
        'Color', Cr, 'HorizontalAlignment','center','VerticalAlignment','bottom');
end
plot(S.R_HB*cos(th), S.R_HB*sin(th), '-', 'Color', 'k', 'LineWidth', 1.2);

% Fallo antes y despues de la maniobra
b0 = B0.b;
m1 = S.nav_sweep(2).miss_req;                 % caso sigma = 40 m
b1 = b0 * (m1/norm(b0));
plot([0 b0(1)], [0 b0(2)], '-',  'Color', Cb, 'LineWidth', 1.2);
plot(b0(1), b0(2), 'o', 'Color', Cb, 'MarkerFaceColor', Cb, 'MarkerSize', 5);
plot([0 b1(1)], [0 b1(2)], '-',  'Color', Cg, 'LineWidth', 1.2);
plot(b1(1), b1(2), '^', 'Color', Cg, 'MarkerFaceColor', Cg, 'MarkerSize', 5);
text(b0(1)+18, b0(2), sprintf('sin maniobra\n%.0f m', norm(b0)), ...
    'FontSize', 6.5, 'Color', Cb);
text(b1(1)-18, b1(2), sprintf('tras %.0f mm/s\n%.0f m', S.nav_sweep(2).dv*1e3, m1), ...
    'FontSize', 6.5, 'Color', Cg, 'HorizontalAlignment','right');

axis equal; grid on; set(gca,'GridAlpha',0.12,'Layer','top');
xlabel('\xi  [m]'); ylabel('\zeta  [m]');
xlim([-250 450]); ylim([-250 350]);
savefig_ieee(f, 'figC_cam_bplane', OUT, W, H);

fprintf('CAM_FIGURAS_OK\n');

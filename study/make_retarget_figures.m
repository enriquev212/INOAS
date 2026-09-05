%MAKE_RETARGET_FIGURES Figuras de la arquitectura que funciona.
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
E = load(fullfile(DATA,'test_cam_retarget.mat'));
S = load(fullfile(DATA,'cam_retarget_sweep.mat'));

W = 3.5; H = 2.6; FS = 8;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',FS);
set(groot,'defaultLineLineWidth',1.0);
Cb=[0 .45 .70]; Cr=[.84 .19 .15]; Cg=[0 .55 .40]; Ck=[.45 .45 .45];

%% ---- Fig D: ejecucion de la maniobra ----------------------------------
f = figure('Visible','off');
tmin = (E.tv - E.SC.t_tca)/60;              % minutos respecto al TCA
yyaxis left
plot(tmin, E.un*1e6, '-', 'Color', Cb);
ylabel('Aceleracion comandada  [\mum/s^2]');
set(gca,'YColor',Cb); ylim([0 70]);
yyaxis right
plot(tmin, E.err, '-', 'Color', Cg);
ylabel('Error de seguimiento  [m]');
set(gca,'YColor',Cg); ylim([0 5]);
xlabel('Tiempo respecto al encuentro  [min]');
xline((E.PLAN.t_burn - E.SC.t_tca)/60, '--', 'Color', Ck);
text((E.PLAN.t_burn - E.SC.t_tca)/60 + 2, 62, 'maniobra', 'FontSize', 6.5, 'Color', Ck);
xline(0, ':', 'Color', Cr);
text(-3, 62, 'TCA', 'FontSize', 6.5, 'Color', Cr, 'HorizontalAlignment','right');
grid on; set(gca,'GridAlpha',.12,'Layer','top');
savefig_ieee(f, 'figD_cam_execution', OUT, W, H);

%% ---- Fig E: coste frente a calidad de navegacion ----------------------
f = figure('Visible','off');
sg = [S.SW.sig]; dv = [S.SW.dv]*1e3; dt = [S.SW.dt];
yyaxis left
plot(sg, dv, 'o-', 'Color', Cb, 'MarkerFaceColor', Cb, 'MarkerSize', 4);
ylabel('\Deltav de la maniobra  [mm/s]'); ylim([0 70]);
set(gca,'YColor',Cb);
yyaxis right
plot(sg, dt, 's--', 'Color', Cr, 'MarkerFaceColor','w', 'MarkerSize', 4);
ylabel('Separacion exigida  [m]'); ylim([0 900]);
set(gca,'YColor',Cr);
xlabel('Incertidumbre de navegacion en el encuentro, 1\sigma  [m]');
legend({'\Deltav','separacion a 3\sigma'}, 'Location','northwest', ...
    'FontSize',6.5,'Box','off');
grid on; set(gca,'GridAlpha',.12,'Layer','top');
savefig_ieee(f, 'figE_cam_nav_cost', OUT, W, H);

fprintf('RETARGET_FIGURAS_OK\n');

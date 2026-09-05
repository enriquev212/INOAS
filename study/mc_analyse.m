%MC_ANALYSE Lectura de la campana Monte Carlo.
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

f = dir(fullfile(DATA,'mc_cam_0*.mat'));
A = [];
for k = 1:numel(f)
    S = load(fullfile(f(k).folder, f(k).name));
    A = [A, S.M]; %#ok<AGROW>
end
SIG = unique([A.sig]);
nGeom = numel(A)/numel(SIG);
fprintf('%d geometrias x %d niveles de sigma = %d puntos\n\n', nGeom, numel(SIG), numel(A));

vr   = [A.vrel]/1e3;  sens = [A.sens];  dv = [A.dv]*1e3;  sg = [A.sig];
m0   = [A.miss0];     dt   = [A.dtarget];  ok = [A.ok];

%% ---- 1) cobertura -------------------------------------------------------
fprintf('--- Cobertura del muestreo ---\n');
fprintf('  v_rel        : %.2f a %.2f km/s (mediana %.2f)\n', min(vr), max(vr), median(vr));
fprintf('  miss inicial : %.0f a %.0f m\n', min(m0), max(m0));
fprintf('  casos sin solucion en la rejilla de dv: %d de %d (%.1f %%)\n\n', ...
    sum(~ok), numel(ok), 100*mean(~ok));

%% ---- 2) el resultado que sostiene la tesis del paper --------------------
fprintf('--- Coste de la maniobra frente a la incertidumbre de navegacion ---\n');
fprintf('  %8s %10s %10s %10s %10s %12s\n', ...
    'sigma', 'p10', 'mediana', 'p90', 'media', '%% sin manio.');
STAT = struct('sig',{},'p10',{},'p50',{},'p90',{},'mean',{},'fzero',{});
for s = SIG
    sel = sg == s & ok;
    d = abs(dv(sel));
    q = prctile(d, [10 50 90]);
    fprintf('  %6.0f m %9.1f %10.1f %10.1f %10.1f %11.1f\n', ...
        s, q(1), q(2), q(3), mean(d), 100*mean(d < 1e-6));
    STAT(end+1) = struct('sig',s,'p10',q(1),'p50',q(2),'p90',q(3), ...
        'mean',mean(d),'fzero',mean(d < 1e-6)); %#ok<SAGROW>
end

r = STAT(end).p50 / STAT(1).p50;
fprintf('\n  razon mediana(250 m)/mediana(12 m) = %.2f\n', r);

%% ---- 3) efectividad del impulso tangencial ------------------------------
fprintf('\n--- Efectividad de un impulso tangencial segun la geometria ---\n');
sg1 = sens(sg == SIG(1));                 % una entrada por geometria
fprintf('  |sensibilidad| : mediana %.2f m por mm/s, rango %.2f a %.2f\n', ...
    median(abs(sg1)), min(abs(sg1)), max(abs(sg1)));
fprintf('  geometrias que exigen impulso RETROGRADO: %.1f %%\n', 100*mean(sg1 < 0));
fprintf('  geometrias casi ciegas (|sens| < 1 m/mm/s): %.1f %%\n', 100*mean(abs(sg1) < 1));

%% ---- 4) figura: bandas de dispersion -----------------------------------
W = 3.5; H = 2.6;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',8);
set(groot,'defaultLineLineWidth',1.0);
Cb=[0 .45 .70]; Ck=[.45 .45 .45];

fg = figure('Visible','off');
xs = [STAT.sig];
p10 = [STAT.p10]; p50 = [STAT.p50]; p90 = [STAT.p90];
fill([xs fliplr(xs)], [p10 fliplr(p90)], Cb, 'FaceAlpha', .16, 'EdgeColor','none');
hold on
plot(xs, p50, 'o-', 'Color', Cb, 'MarkerFaceColor', Cb, 'MarkerSize', 4);
xlabel('Incertidumbre de navegacion en el encuentro, 1\sigma  [m]');
ylabel('\Deltav de la maniobra  [mm/s]');
legend({'p10 - p90 sobre la geometria','mediana'}, 'Location','northwest', ...
    'FontSize',6.5,'Box','off');
text(0.97, 0.06, sprintf('%d geometrias', nGeom), 'Units','normalized', ...
    'HorizontalAlignment','right', 'FontSize',6.5, 'Color',Ck);
grid on; set(gca,'GridAlpha',.12,'Layer','top');
savefig_ieee(fg, 'figF_mc_nav_cost', OUT, W, H);

%% ---- 5) figura: efectividad frente a la velocidad relativa -------------
fg = figure('Visible','off');
vr1 = vr(sg == SIG(1));
scatter(vr1, sg1, 6, Cb, 'filled', 'MarkerFaceAlpha', .35);
hold on; yline(0, '-', 'Color', Ck);
xlabel('Velocidad relativa en el encuentro  [km/s]');
ylabel('Sensibilidad  [m de fallo por mm/s]');
grid on; set(gca,'GridAlpha',.12,'Layer','top');
savefig_ieee(fg, 'figG_mc_sensitivity', OUT, W, H);

save(fullfile(DATA,'mc_summary.mat'), 'STAT', 'nGeom', 'SIG');
fprintf('\nMC_ANALYSE_OK\n');

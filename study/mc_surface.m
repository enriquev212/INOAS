%MC_SURFACE El resultado del paper como superficie, no como un par de numeros.
%
% El coste de la maniobra depende de DOS incertidumbres: la propia y la del
% objeto. Con efemerides de catalogo la del objeto domina y la calidad de
% navegacion apenas importa; con efemerides frescas la relacion se invierte.
% Reportar un solo par de numeros esconde justo eso, y la magnitud que se
% reporte quedaria fijada por una covarianza elegida a mano.
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
for k = 1:numel(f); S = load(fullfile(f(k).folder,f(k).name)); A = [A, S.M]; end %#ok<AGROW>
SN = unique([A.sig]);  SO = unique([A.sig_obj]);
nGeom = numel(A)/(numel(SN)*numel(SO));
fprintf('%d geometrias x %d sigma_nav x %d sigma_obj = %d puntos\n\n', ...
    nGeom, numel(SN), numel(SO), numel(A));

sg = [A.sig]; so = [A.sig_obj]; dv = abs([A.dv])*1e3; ok = [A.ok];

MED = nan(numel(SO), numel(SN));
for i = 1:numel(SO)
    for j = 1:numel(SN)
        sel = so == SO(i) & sg == SN(j) & ok;
        MED(i,j) = median(dv(sel));
    end
end

fprintf('Mediana del delta-v [mm/s]\n');
fprintf('  %-14s', 'sigma_obj \\ nav');
fprintf('%9.1f', SN); fprintf('   razon\n');
for i = 1:numel(SO)
    fprintf('  %10.0f m  ', SO(i));
    fprintf('%9.1f', MED(i,:));
    fprintf('%8.2f\n', MED(i,end)/MED(i,1));
end

% Los dos regimenes MEDIDOS de navegacion
[~, jf] = min(abs(SN - 4.8));  [~, js] = min(abs(SN - 32.1));
fprintf('\nEntre los dos regimenes medidos (%.1f y %.1f m de radio 3D):\n', SN(jf), SN(js));
for i = 1:numel(SO)
    fprintf('  sigma_obj = %3.0f m -> %.1f vs %.1f mm/s  (%+.1f %%)\n', ...
        SO(i), MED(i,jf), MED(i,js), 100*(MED(i,js)/MED(i,jf)-1));
end

%% ---- figura ------------------------------------------------------------
W = 3.5; H = 2.6;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',8);
set(groot,'defaultLineLineWidth',1.0);
cols = [0 .45 .70; 0 .55 .40; .90 .55 .10; .84 .19 .15];
mk   = {'o','s','^','d'};

fg = figure('Visible','off');
for i = 1:numel(SO)
    semilogx(SN, MED(i,:), '-', 'Color', cols(i,:), 'Marker', mk{i}, ...
        'MarkerFaceColor', cols(i,:), 'MarkerSize', 4); hold on
end
xline(4.8,  ':', 'Color', [.45 .45 .45]);
xline(32.1, ':', 'Color', [.45 .45 .45]);
text(4.8, 105, ' GNSS fresco', 'FontSize', 6, 'Color', [.45 .45 .45], 'Rotation', 90);
text(32.1, 105, ' saturado', 'FontSize', 6, 'Color', [.45 .45 .45], 'Rotation', 90);
xlabel('Incertidumbre de navegacion en el encuentro  [m, radio 3D]');
ylabel('\Deltav mediano de la maniobra  [mm/s]');
lg = arrayfun(@(x) sprintf('\\sigma_{obj} = %d m', x), SO, 'UniformOutput', false);
legend(lg, 'Location','northwest', 'FontSize',6.5, 'Box','off');
xlim([4 300]); ylim([0 140]);
grid on; set(gca,'GridAlpha',.12,'Layer','top');
savefig_ieee(fg, 'figJ_mc_surface', OUT, W, H);

save(fullfile(DATA,'mc_surface.mat'), 'MED', 'SN', 'SO', 'nGeom');
fprintf('\nMC_SURFACE_OK\n');

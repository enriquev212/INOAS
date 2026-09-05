%CHAIN_ANALYSE La cadena completa, medida de punta a punta.
%
%   ciclo de trabajo del GNSS  ->  sigma de navegacion  ->  margen exigido  ->  delta-v
%
% Hasta ahora las dos mitades estaban medidas por separado y el eslabon central
% era una hipotesis. Aqui la ley sigma(hueco) sale del modelo completo y los
% costes salen del Monte Carlo evaluado en ESOS sigmas, no en valores elegidos.
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

N = load(fullfile(DATA,'nav_chain.mat'));  NAV = N.NAV;
f = dir(fullfile(DATA,'mc_cam_0*.mat'));
A = [];
for k = 1:numel(f); S = load(fullfile(f(k).folder,f(k).name)); A = [A, S.M]; end %#ok<AGROW>
SIG = unique([A.sig]);  nGeom = numel(A)/numel(SIG);

SIG_FRESH = 4.8;      % medido: justo tras una correccion de GNSS
SIG_SAT   = 32.1;     % medido: filtro saturado en propagacion

fprintf('=== CADENA COMPLETA ===\n\n');
fprintf('1) Ciclo de trabajo, medido sobre el modelo completo (%d s)\n', NAV.tf);
fprintf('   GNSS activo          : %.2f %% del tiempo\n', 100*NAV.onFrac);
fprintf('   hueco de propagacion : mediana %.0f s, maximo %.0f s\n\n', ...
    median(NAV.gap), max(NAV.gap));

fprintf('2) Incertidumbre de navegacion resultante\n');
fprintf('   con correccion fresca : %.1f m\n', SIG_FRESH);
fprintf('   en saturacion         : %.1f m\n', SIG_SAT);
fprintf('   (satura porque los sensores auxiliares acotan la deriva: es lo que\n');
fprintf('    la arquitectura promete, y valida el "<40 m" publicado)\n\n');

fprintf('3) Coste de la maniobra en ESOS dos regimenes (%d geometrias)\n', nGeom);
fprintf('   %-24s %10s %10s %10s %12s\n','regimen','p10','mediana','p90','sin maniobra');
res = struct('name',{},'sig',{},'p50',{},'p10',{},'p90',{},'fzero',{});
for pair = {{'GNSS recien corregido', SIG_FRESH}, {'propagacion saturada', SIG_SAT}}
    nm = pair{1}{1}; sv = pair{1}{2};
    sel = abs([A.sig] - sv) < 1e-6 & [A.ok];
    d = abs([A(sel).dv])*1e3;
    q = prctile(d,[10 50 90]);
    fprintf('   %-24s %10.1f %10.1f %10.1f %11.1f %%\n', nm, q(1), q(2), q(3), 100*mean(d<1e-6));
    res(end+1) = struct('name',nm,'sig',sv,'p50',q(2),'p10',q(1),'p90',q(3), ...
                        'fzero',mean(d<1e-6)); %#ok<SAGROW>
end

dPct  = 100*(res(2).p50/res(1).p50 - 1);
dMan  = 100*(res(1).fzero - res(2).fzero);
fprintf('\n   Programar una correccion de GNSS antes del encuentro:\n');
fprintf('     ahorra un %.0f %% del delta-v de la maniobra\n', dPct/(1+dPct/100));
fprintf('     y evita la maniobra por completo en %.1f puntos mas de los casos\n', dMan);

%% ---- figura de la cadena ----------------------------------------------
W = 3.5; H = 2.6;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',8);
set(groot,'defaultLineLineWidth',1.0);
Cb=[0 .45 .70]; Cr=[.84 .19 .15]; Cg=[0 .55 .40]; Ck=[.45 .45 .45];

fg = figure('Visible','off');
% Panel unico: sigma frente al hueco (medido) con los dos regimenes marcados
ctr = NAV.edges(1:end-1) + diff(NAV.edges)/2;
ok  = NAV.cnt > 0;
plot(ctr(ok), NAV.law(ok), 'o-', 'Color', Cb, 'MarkerFaceColor', Cb, 'MarkerSize', 4);
hold on
yline(SIG_SAT, '--', 'Color', Ck);
text(295, SIG_SAT+1.4, sprintf('saturacion: %.1f m', SIG_SAT), ...
    'FontSize', 6.5, 'Color', Ck, 'HorizontalAlignment','right');
plot(ctr(1), NAV.law(1), 's', 'Color', Cg, 'MarkerFaceColor', Cg, 'MarkerSize', 6);
text(ctr(1)+12, NAV.law(1)-1.5, sprintf('correccion fresca: %.1f m', NAV.law(1)), ...
    'FontSize', 6.5, 'Color', Cg);
xlabel('Tiempo desde la ultima correccion de GNSS  [s]');
ylabel('Incertidumbre de navegacion, 1\sigma  [m]');
xlim([0 300]); ylim([0 38]);
grid on; set(gca,'GridAlpha',.12,'Layer','top');
savefig_ieee(fg, 'figH_nav_law', OUT, W, H);

save(fullfile(DATA,'chain_summary.mat'), 'res', 'NAV', 'nGeom', ...
     'SIG_FRESH', 'SIG_SAT');
fprintf('\nCHAIN_OK\n');

%MAKE_FIGURES Figuras para el paper de IEEE Aerospace.
% Formato de columna simple IEEE: 3.5 in de ancho, tipografia de 8 pt.
% Salida en PDF vectorial (para LaTeX) y PNG a 300 dpi (para revisar).
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
OUT = FIG_DIR;
if ~exist(OUT,'dir'); mkdir(OUT); end
DATA = OUT_DIR;

W = 3.5; H = 2.6;                      % pulgadas, columna simple IEEE
FS = 8;                                % pt
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', FS);
set(groot, 'defaultLineLineWidth', 1.0);

% Paleta segura para daltonismo e impresion en gris
C.blue = [0.00 0.45 0.70];
C.red  = [0.84 0.19 0.15];
C.grn  = [0.00 0.55 0.40];
C.orn  = [0.90 0.55 0.10];
C.gry  = [0.45 0.45 0.45];

fprintf('cargando resultados...\n');
files = dir(fullfile(DATA, 'expB_*.mat'));
RES_ALL = cell(1, numel(files));
for kf = 1:numel(files)
    S = load(fullfile(files(kf).folder, files(kf).name));
    RES_ALL{kf} = S.RES;
end
Th   = cellfun(@(r) r.Th, RES_ALL);
dv   = cellfun(@(r) r.dv_total, RES_ALL);
upk  = cellfun(@(r) r.u_peak, RES_ALL);
hh   = cellfun(@(r) r.h, RES_ALL);
umax = RES_ALL{1}.u_max;
[Th, ord] = sort(Th); dv = dv(ord); upk = upk(ord); hh = hh(ord); RES_ALL = RES_ALL(ord);

%% ===== Fig 1: coste de la maniobra y de computo frente al horizonte ======
fprintf('Fig 1: delta-v y coste computacional vs horizonte\n');
f = figure('Visible','off');
yyaxis left
i5 = hh == 5; i10 = hh == 10; i3 = hh == 3;
plot(Th(i5|i3), dv(i5|i3), 'o-', 'Color', C.blue, 'MarkerFaceColor', C.blue, 'MarkerSize', 4);
hold on
plot(Th(i10), dv(i10), 's--', 'Color', C.blue, 'MarkerFaceColor','w', 'MarkerSize', 4);
% Cota teorica ideal: delta-v impulsivo para un desplazamiento fijo
d_target = 433;
Thc = linspace(min(Th), max(Th), 100);
plot(Thc, d_target./(3*Thc), ':', 'Color', C.gry);
ylabel('Total \Deltav over 400-950 s  [m/s]');
ylim([0 6]);
set(gca, 'YColor', C.blue);

yyaxis right
wc5 = [3.61 3.04 9.73];  th5 = [375 500 750];
plot(th5, wc5, '^-', 'Color', C.red, 'MarkerFaceColor', C.red, 'MarkerSize', 4);
ylabel('Worst-case solve time  [s]');
ylim([0 12]);
set(gca, 'YColor', C.red);

xlabel('Prediction horizon T_h  [s]');
xlim([330 1050]);
legend({'\Deltav, h = 3 and 5 s', '\Deltav, h = 10 s', ...
        'impulsive bound for a 433 m offset', 'solve time (worst)'}, ...
       'Location','northwest', 'FontSize', 6.5, 'Box','off');
grid on; set(gca,'GridAlpha',0.12, 'Layer','top');
savefig_ieee(f, 'fig1_dv_vs_horizon', OUT, W, H);

%% ===== Fig 2: perfil de la keep-out zone, antes y despues ================
fprintf('Fig 2: perfil de d_safe (mallado vs tiempo)\n');
P_nav = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);
casos = {struct('h',5,'Np',100), struct('h',10,'Np',50), struct('h',4,'Np',125)};
prof = struct('legacy', {cell(1,3)}, 'vanloan', {cell(1,3)}, 'tt', {cell(1,3)});
for modo = ["legacy","vanloan"]
    for kc = 1:3
        c = casos{kc};
        setpref('inoas','skipBatchClear',true);
        setpref('inoas','mpcTuneConfig', struct('h',c.h,'Np',c.Np));
        mpcQuiet = true;
        evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
        mpcQuiet = true;
        mpcVanLoanQ = strcmp(modo,"vanloan");
        % Sin techo de sigma para ESTA figura. El techo (sigma_nav_max) es una
        % propiedad del sistema de navegacion, no de la formulacion, y satura
        % las dos curvas al mismo valor: enmascararia justo lo que la figura
        % demuestra, que es como se acumula el ruido de proceso. En la
        % configuracion de vuelo el techo actua y ambas quedan en ~246 m.
        sigma_nav_max = inf;
        safetyCost = 0.2;   % el valor con el que se caracterizo el efecto
        dsafeSnapshotTimesMpc = 600;
        clear MPC_INOAS mpc_dsafe_snapshot_profile
        ik = min(floor(600/h)+1, Ntimesteps);
        xt = r_p_full((ik-1)*6 + (1:6)) + [30;-20;15;0.02;-0.01;0.01];
        MPC_INOAS(xt, P_nav, 600);
        pr = evalin('base','mpc_dsafe_snapshot_profile');
        prof.(modo){kc} = pr(:,end);
        prof.tt{kc} = (1:numel(pr(:,end)))' * c.h;
    end
end

save(fullfile(OUT_DIR,'koz_profiles.mat'), '-struct', 'prof');

f = figure('Visible','off');
sty = {'-','--',':'};
lbl = {'h = 5 s, N_p = 100', 'h = 10 s, N_p = 50', 'h = 4 s, N_p = 125'};
hL = gobjects(1,6);
for kc = 1:3
    hL(kc) = plot(prof.tt{kc}, prof.legacy{kc}, sty{kc}, 'Color', C.red); hold on
end
% Las tres curvas corregidas caen una encima de otra: ese es el resultado.
% Marcadores distintos y desfasados para que la superposicion se vea.
mk = {'o','s','^'};
for kc = 1:3
    idx = round(linspace(1, numel(prof.tt{kc}), 9));
    idx = idx(1 + mod(kc-1, 3) : 3 : end);
    hL(3+kc) = plot(prof.tt{kc}, prof.vanloan{kc}, '-', 'Color', C.grn, ...
        'Marker', mk{kc}, 'MarkerIndices', idx, 'MarkerSize', 4, ...
        'MarkerFaceColor', 'w');
end
xlabel('Time into the prediction horizon  [s]');
ylabel('Inflated keep-out radius  d_{safe}  [m]');
xlim([0 500]);
legend(hL([1 2 3 4 5 6]), [strcat({'per-step: '}, lbl), ...
        strcat({'time-consistent: '}, lbl)], ...
       'Location','northwest', 'FontSize', 6, 'Box','off', 'NumColumns', 1);
grid on; set(gca,'GridAlpha',0.12, 'Layer','top');
savefig_ieee(f, 'fig2_koz_profile', OUT, W, H);

%% ===== Fig 3: separacion al debris frente al radio robusto ===============
fprintf('Fig 3: separacion al debris\n');
f = figure('Visible','off');
b = RES_ALL{1}.B;
plot(b.t, b.ddeb, '-', 'Color', C.blue); hold on
plot(b.t, b.dsafe, '--', 'Color', C.red);
plot(b.t, 150*ones(size(b.t)), ':', 'Color', C.gry);
[dmin, imin] = min(b.ddeb);
plot(b.t(imin), dmin, 'o', 'Color', C.blue, 'MarkerFaceColor', C.blue, 'MarkerSize', 4);
text(b.t(imin)+10, dmin+95, sprintf('%.0f m', dmin), 'FontSize', 6.5, ...
    'Color', C.blue, 'HorizontalAlignment','left');
xlabel('Time  [s]'); ylabel('Distance to debris  [m]');
xlim([730 870]); ylim([0 1400]);
% Anotar el margen conseguido, que es lo que sostiene la afirmacion de seguridad
plot([b.t(imin) b.t(imin)], [b.dsafe(imin) dmin], '-', 'Color', C.gry);
text(b.t(imin)-6, (dmin + b.dsafe(imin))/2, sprintf('margin %.0f m', dmin - b.dsafe(imin)), ...
    'FontSize', 6.5, 'Color', C.gry, 'HorizontalAlignment','right');
legend({'achieved separation', 'robust radius d_{safe}(t)', 'nominal radius d_0'}, ...
       'Location','north', 'FontSize', 6.5, 'Box','off');
grid on; set(gca,'GridAlpha',0.12, 'Layer','top');
savefig_ieee(f, 'fig3_debris_separation', OUT, W, H);

%% ===== Fig 4: esfuerzo de control frente al limite del propulsor =========
fprintf('Fig 4: esfuerzo de control y limite del propulsor\n');
f = figure('Visible','off');
cols = {C.blue, C.grn, C.orn, C.red};
sel = [1 2 4 5];        % Th = 375, 500(h5), 750(h5), 750(h10)
nm = cell(1, numel(sel));
for ks = 1:numel(sel)
    r = RES_ALL{sel(ks)};
    plot(r.B.t, vecnorm(r.B.u,2,1), '-', 'Color', cols{ks}); hold on
    nm{ks} = sprintf('h=%g, N_p=%d', r.h, r.Np);
end
yline(umax, 'k--');
text(940, umax*0.93, sprintf('u_{max} = %.4f m/s^2 (1 N / 24 kg)', umax), ...
    'FontSize', 6.5, 'HorizontalAlignment','right', 'VerticalAlignment','top');
xlabel('Time  [s]'); ylabel('Commanded acceleration  ||u||  [m/s^2]');
xlim([400 950]); ylim([0 0.055]);
legend(nm, 'Location','northwest', 'FontSize', 6.5, 'Box','off', 'NumColumns', 2);
grid on; set(gca,'GridAlpha',0.12, 'Layer','top');
savefig_ieee(f, 'fig4_control_effort', OUT, W, H);

%% ===== Fig 5: coste computacional, antes y despues ======================
fprintf('Fig 5: distribucion del tiempo de solve\n');
A = load(fullfile(DATA,'bench_consec_fmincon_sqp.mat'));
B2 = load(fullfile(DATA,'bench_consec_quadprog_as.mat'));
f = figure('Visible','off');
labs = cell(1, numel(A.filas)); xa = []; ga = []; xb = []; gb = [];
for kf = 1:numel(A.filas)
    % Una sola linea: un salto aqui parte la etiqueta en dos ticks distintos.
    labs{kf} = sprintf('h=%g, N_p=%d', A.filas{kf}.h, A.filas{kf}.Np);
    xa = [xa, A.filas{kf}.ts];  ga = [ga, kf*ones(1,numel(A.filas{kf}.ts))];
    xb = [xb, B2.filas{kf}.ts]; gb = [gb, kf*ones(1,numel(B2.filas{kf}.ts))];
end
for kf = 1:numel(A.filas)
    ya = A.filas{kf}.ts;  yb = B2.filas{kf}.ts;
    plot((kf-0.16)*ones(size(ya)), ya, '.', 'Color', [C.red 0.28], 'MarkerSize', 3); hold on
    plot((kf+0.16)*ones(size(yb)), yb, '.', 'Color', [C.grn 0.28], 'MarkerSize', 3);
    plot(kf-0.16, median(ya), '_', 'Color', C.red, 'MarkerSize', 13, 'LineWidth', 1.4);
    plot(kf+0.16, median(yb), '_', 'Color', C.grn, 'MarkerSize', 13, 'LineWidth', 1.4);
    plot([kf-0.34 kf+0.34], A.filas{kf}.h*[1 1], 'k--');
end
set(gca, 'YScale','log', 'XTick', 1:numel(A.filas), 'XTickLabel', labs, ...
    'XTickLabelRotation', 20, 'TickLabelInterpreter','tex');
ylabel('Solve time  [s]'); ylim([5e-3 4e2]);
xlim([0.5 numel(A.filas)+0.5]);
text(0.62, 1.7e2, 'dashed: real-time deadline = h', 'FontSize', 6.5);
legend({'fmincon (SQP)','quadprog (active-set)'}, 'Location','southwest', ...
       'FontSize', 6.5, 'Box','off');
grid on; set(gca,'GridAlpha',0.12, 'Layer','top');
savefig_ieee(f, 'fig5_solve_time', OUT, W, H);

fprintf('\nFIGURAS_OK -> %s\n', OUT);

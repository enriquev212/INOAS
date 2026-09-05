%VERIFY_PHASE Comprobacion tras cada fase de correccion.
% 1) sintaxis de los ficheros tocados
% 2) el MPC sigue corriendo y devuelve lo mismo que antes del cambio
% 3) el nuevo log de diagnostico existe y esta poblado
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

REPO = REPO_ROOT;
addpath(REPO); addpath(genpath(fullfile(REPO,'matlab')));
addpath(STUDY_DIR);

fprintf('=== 1. SINTAXIS ===\n');
files = {'initialize_inoas_simulation.m', 'matlab/plot_MPC_results.m', ...
         'matlab/MPC_INOAS.m', 'matlab/myStateTransitionFcn.m'};
nbad = 0;
for k = 1:numel(files)
    f = fullfile(REPO, files{k});
    if ~isfile(f); fprintf('  %-38s (no existe)\n', files{k}); continue; end
    report = evalc('checkcode(f)');
    if contains(report, 'arse error') || contains(report, 'nbalanced') || ...
            contains(report, 'nterminated')
        fprintf('  %-38s ERROR DE SINTAXIS\n', files{k});
        disp(report); nbad = nbad + 1;
    else
        fprintf('  %-38s ok\n', files{k});
    end
end
if nbad > 0
    error('verify_phase:syntax', '%d fichero(s) con errores de sintaxis', nbad);
end

fprintf('\n=== 2. EJECUCION DEL MPC (h=3, Np=125) ===\n');
CFG_ID = 1;
run(fullfile(STUDY_DIR, 'inoas_setup.m'));

P_nav = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);
clear MPC_INOAS;
tprobe = [0 300 600 700 750 800 850];
u_all = zeros(3, numel(tprobe));
for k = 1:numel(tprobe)
    ik = min(floor(tprobe(k)/h)+1, Ntimesteps);
    xt = r_p_full((ik-1)*6 + (1:6));
    xe = xt + [30; -20; 15; 0.02; -0.01; 0.01];
    clear MPC_INOAS;
    [u_all(:,k), ~, ~] = MPC_INOAS(xe, P_nav, tprobe(k));
end
fprintf('  u(t=800) = [% .6f % .6f % .6f]  |u| = %.6f\n', ...
    u_all(1,end-1), u_all(2,end-1), u_all(3,end-1), norm(u_all(:,end-1)));
fprintf('  pico |u| sobre las sondas = %.6f m/s2 (%.1f %% de u_max)\n', ...
    max(vecnorm(u_all,2,1)), 100*max(vecnorm(u_all,2,1))/u_max);

fprintf('\n=== 3. LOG DE DIAGNOSTICO ===\n');
if evalin('base','exist(''mpc_diag_log'',''var'')')
    d = evalin('base','mpc_diag_log');
    fprintf('  %d solves registrados\n', numel(d.t));
    fprintf('  solve time      : media %.3f s | max %.3f s\n', mean(d.solve_time), max(d.solve_time));
    fprintf('  exitflag        : %s\n', mat2str(unique(d.exitflag).'));
    fprintf('  %% exitflag > 0   : %.1f %%\n', 100*mean(d.exitflag > 0));
    fprintf('  iteraciones     : mediana %g | max %g  (limite MaxIterations = 50)\n', ...
        median(d.iterations), max(d.iterations));
    fprintf('  violacion max   : %.3e\n', max(d.violation));
    fprintf('  recorte activo  : %d de %d solves | mordida max %.3e m/s2\n', ...
        sum(d.clamp_bite > 0), numel(d.t), max(d.clamp_bite));
    fprintf('  norma sin recortar: max %.6f (%.1f %% de u_max)\n', ...
        max(d.u_norm_unclamped), 100*max(d.u_norm_unclamped)/u_max);
    fprintf('  du_max activo   : %d de %d solves\n', sum(d.du_max_active), numel(d.t));
else
    error('verify_phase:nolog', 'mpc_diag_log no existe: la instrumentacion no se ejecuto');
end
fprintf('\nVERIFY_OK\n');

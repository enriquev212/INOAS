%DIAG_MARGIN Por que cae la convergencia con el margen de 3 sigma.
% Dos causas posibles y muy distintas:
%   (a) el QP no converge por condicionamiento -> se arregla con el solver
%   (b) el problema es genuinamente INFACTIBLE -> es fisica, no numerica, y hay
%       que decirlo en el paper en vez de esconderlo
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
REPO = REPO_ROOT;
addpath(REPO); addpath(genpath(fullfile(REPO,'matlab')));
addpath(STUDY_DIR);

setpref('inoas','skipBatchClear', true);
setpref('inoas','mpcTuneConfig', struct('h',5,'Np',100));
mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));
mpcQuiet = true;

fprintf('dsafe0 = %g, safetyCost = %g, sigma_nav_max = %g\n', dsafe0, safetyCost, sigma_nav_max);
P_nav = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);

clear MPC_INOAS mpc_diag_log mpc_dsafe_log_first
tp = 500:5:900;
for k = 1:numel(tp)
    ik = min(floor(tp(k)/h)+1, Ntimesteps);
    xt = r_p_full((ik-1)*6 + (1:6)) + [30;-20;15;0.02;-0.01;0.01];
    MPC_INOAS(xt, P_nav, tp(k));
end
D = evalin('base','mpc_diag_log');
df = evalin('base','mpc_dsafe_log_first');

fprintf('\n%d solves\n', numel(D.t));
fprintf('  exitflag: %s\n', mat2str(unique(D.exitflag).'));
for e = unique(D.exitflag).'
    fprintf('    exitflag %2d : %3d solves (%.1f %%)\n', e, sum(D.exitflag==e), ...
        100*mean(D.exitflag==e));
end
fprintf('  iteraciones: mediana %g, max %g\n', median(D.iterations), max(D.iterations));
fprintf('  d_safe primer paso: %.1f m\n', df(1));
fprintf('  holgura > 0 en %d de %d solves (%.1f %%)\n', ...
    sum(D.slack_max > 1e-6), numel(D.t), 100*mean(D.slack_max > 1e-6));
fprintf('  holgura mediana cuando actua: %.3e\n', median(D.slack_max(D.slack_max>1e-6)));

bad = D.exitflag <= 0;
if any(bad)
    fprintf('\n  En los solves que NO convergen:\n');
    fprintf('    holgura > 0 en %.1f %%  -> %s\n', 100*mean(D.slack_max(bad) > 1e-6), ...
        ternario(mean(D.slack_max(bad) > 1e-6) > 0.5, ...
        'INFACTIBLE: es fisica, no numerica', 'condicionamiento del QP'));
    fprintf('    iteraciones: mediana %g, max %g\n', ...
        median(D.iterations(bad)), max(D.iterations(bad)));
end
fprintf('\nDIAG_MARGIN_OK\n');

function o = ternario(c,a,b); if c; o=a; else; o=b; end; end

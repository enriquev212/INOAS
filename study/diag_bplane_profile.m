%DIAG_BPLANE_PROFILE Donde se gasta el delta-v en el modo plano-B.
% Hipotesis: el MPC no gasta el propelente en una maniobra unica, sino que
% empuja para abrir la distancia y luego vuelve a empujar para regresar a la
% referencia. El termino de seguimiento pelea contra el de evitacion.
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

D = load(fullfile(OUT_DIR,'test_bplane_mpc.mat'));
R = D.RES.bplane_tca;
h = D.H_MPC;
t = R.t;  u = R.u;

dv_step = u*h;
dv_cum  = cumsum(dv_step);
t_tca   = D.SC.t_tca;

fprintf('TCA en t = %.0f s. El encuentro entra en el horizonte en t = %.0f s.\n', ...
    t_tca, t_tca - h*D.NP_MPC);
fprintf('delta-v total: %.1f mm/s\n\n', dv_cum(end)*1e3);

fprintf('%8s %12s %12s %12s\n', 't [s]', '|u| [m/s2]', 'dv paso', 'dv acum [mm/s]');
sel = round(linspace(1, numel(t), 18));
for k = sel
    mark = '';
    if t(k) > t_tca - h*D.NP_MPC && t(k) < t_tca; mark = '  <- en horizonte'; end
    if abs(t(k)-t_tca) < h; mark = '  <- TCA'; end
    fprintf('%8.0f %12.3e %12.4f %12.2f%s\n', t(k), u(k), dv_step(k)*1e3, dv_cum(k)*1e3, mark);
end

% Reparto antes / durante / despues
pre  = t <= t_tca - h*D.NP_MPC;
dur  = t > t_tca - h*D.NP_MPC & t <= t_tca;
post = t > t_tca;
fprintf('\nReparto del delta-v:\n');
fprintf('  antes de entrar en el horizonte : %8.1f mm/s (%4.1f %%)\n', ...
    sum(dv_step(pre))*1e3,  100*sum(dv_step(pre))/dv_cum(end));
fprintf('  con el encuentro en el horizonte: %8.1f mm/s (%4.1f %%)\n', ...
    sum(dv_step(dur))*1e3,  100*sum(dv_step(dur))/dv_cum(end));
fprintf('  despues del TCA                 : %8.1f mm/s (%4.1f %%)\n', ...
    sum(dv_step(post))*1e3, 100*sum(dv_step(post))/dv_cum(end));
fprintf('\nDIAG_PROFILE_OK\n');

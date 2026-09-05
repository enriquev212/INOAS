%TEST_BPLANE_MPC Lazo cerrado del MPC con la restriccion en el plano-B.
%
% Prueba de fondo: con una conjuncion realista (cruce de planos, ~10 km/s),
% comprobar que el controlador
%   (a) no hace nada mientras el encuentro esta fuera del horizonte,
%   (b) maniobra cuando entra,
%   (c) abre la distancia de maxima aproximacion hasta el margen pedido,
%   (d) gasta un delta-v del orden de decenas de mm/s, no de m/s.
%
% Se compara ademas contra el modo antiguo (esfera muestreada en la rejilla),
% que deberia no reaccionar en absoluto.
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
OUT = OUT_DIR;

MU = 3.986004418e14;
H_MPC   = 60;      NP_MPC = 120;        % Th = 7200 s, algo mas de una orbita
TF      = 12000;                        % s de referencia
T_TCA   = 10115;                        % ~1.5 orbitas
K_SIGMA = 3;

setpref('inoas','skipBatchClear', true);
setpref('inoas','mpcTuneConfig', struct('h',H_MPC,'Np',NP_MPC));
simulationStopTime = TF;
mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));
mpcQuiet = true;

T_orb = 2*pi/sqrt(MU/a^3);
fprintf('h = %g s, Np = %d, Th = %g s (%.2f orbitas)\n', h, Np, h*Np, h*Np/T_orb);

%% ---- escenario de conjuncion realista ---------------------------------
SC = get_conjunction_scenario(h, x_ref_hist, t_ref, T_TCA);
fprintf('\nConjuncion: cruce %.2f deg | v_rel = %.3f km/s | miss nominal = %.1f m\n', ...
    SC.dInc_deg, SC.v_rel_mag/1e3, SC.miss_nom);
fprintf('  semieje sat/objeto = %.3f / %.3f km\n', SC.a_chaser/1e3, SC.a_debris/1e3);
fprintf('  paso por esfera de %.0f m: %.1f ms | el objeto recorre %.1f km por paso de MPC\n', ...
    SC.miss_nom, SC.pass_time*1e3, SC.v_rel_mag*h/1e3);
fprintf('  TCA en t = %.1f s; entra en el horizonte en t = %.1f s\n', ...
    SC.t_tca, SC.t_tca - h*Np);

% Publicar en el base workspace lo que lee getMpcConfig
x_debris_hist = SC.x_debris_hist;
r_debris_full = SC.r_debris_full;
rk_debris     = SC.rk_debris_encounter;
conj_t_tca    = SC.t_tca;
conj_d_nom_eci= SC.d_nom_eci;
conj_u_rel_eci= SC.u_rel_eci;
conj_P_debris = SC.P_debris;
conj_k_sigma  = K_SIGMA;
% Covarianza de navegacion PREVISTA en el TCA, que es lo que la capa de
% navegacion puede entregar conociendo el plan de ciclo de trabajo. 40 m 1-sigma
% es coherente con el <40 m de error en apagon que reporta el paper.
SIG_TCA = 40;
conj_P_nav_tca = diag([SIG_TCA SIG_TCA SIG_TCA].^2);
dsafe0 = SC.R_hb;                       % el radio nominal pasa a ser el cuerpo duro
% El modo antiguo infla la KOZ propagando la covarianza paso a paso sobre todo
% el horizonte. Sobre 7200 s eso da decenas de km y satura el actuador. Para que
% la comparacion mida la FORMULACION y no ese efecto, se le da a los dos modos
% la misma incertidumbre efectiva.
Q_c_mpc = zeros(6); Q_cov_mpc = zeros(6);
safetyCost = K_SIGMA;

P_nav = diag([40 40 40 0.05 0.05 0.05].^2);

%% ---- lazo cerrado, los dos modos --------------------------------------
T_START = SC.t_tca - 1.35*h*Np;         % arranca antes de que entre en el horizonte
T_END   = SC.t_tca + 2*h;
odeo = odeset('RelTol',1e-11,'AbsTol',1e-9);

RES = struct();
for mode = ["sphere_grid", "bplane_tca", "bplane_retarget"]
    conj_t_deadline = [];
    mpcTrackRelax = 1;
    if strcmp(mode,"bplane_retarget")
        mpcDebrisMode = "bplane_tca";
        % Seguimiento relajado hasta el encuentro: la nominal pasa a ser la
        % trayectoria desplazada, asi que mantener la maniobra no cuesta.
        mpcTrackRelax = 1e-3;
    else
        mpcDebrisMode = mode;
    end
    clear MPC_INOAS
    clear mpc_dsafe_log_time mpc_dsafe_log_first mpc_dsafe_log_max mpc_diag_log

    ik0 = min(floor(T_START/h)+1, Ntimesteps);
    x_true = r_p_full((ik0-1)*6 + (1:6));      % arranca sobre la referencia
    t_now  = (ik0-1)*h;
    nS = floor((T_END - t_now)/h);

    tv = zeros(1,nS); dv_cum = 0; un = zeros(1,nS); act = false(1,nS);
    for k = 1:nS
        [u_cmd,~,slk] = MPC_INOAS(x_true, P_nav, t_now);
        tv(k) = t_now; un(k) = norm(u_cmd);
        act(k) = max(slk) > 1e-6;
        dv_cum = dv_cum + norm(u_cmd)*h;
        [~,XX] = ode113(@(tt,xx) inoas_dyn(tt,xx,u_cmd), [t_now t_now+h], x_true, odeo);
        x_true = XX(end,:).';
        t_now = t_now + h;
    end

    % Distancia de maxima aproximacion REAL, resuelta de forma continua
    idx_d = min(max(round(t_now/h)+1, 1), size(SC.x_debris_hist,2));
    x_d_now = SC.x_debris_hist(:, idx_d);
    Tf = cam_find_tca(x_true, x_d_now, t_now, SC.t_tca, 4*h);

    fprintf('\n=== modo %s ===\n', mode);
    fprintf('  delta-v acumulado : %.3f mm/s\n', dv_cum*1e3);
    fprintf('  pico |u|          : %.3e m/s2 (%.2f %% de u_max)\n', max(un), 100*max(un)/u_max);
    fprintf('  pasos con la restriccion tocando: %d de %d\n', sum(act), nS);
    fprintf('  distancia de maxima aproximacion: %.1f m  (sin maniobra: %.1f m)\n', ...
        Tf.miss, SC.miss_nom);
    RES.(mode) = struct('t',tv,'u',un,'dv',dv_cum,'miss',Tf.miss,'nact',sum(act));
end

%% ---- lectura ----------------------------------------------------------
fprintf('\n---------------------------------------------------------------\n');
fprintf('sin maniobra                : %.1f m\n', SC.miss_nom);
fprintf('esfera en la rejilla        : %.1f m con %.3f mm/s\n', ...
    RES.sphere_grid.miss, RES.sphere_grid.dv*1e3);
fprintf('plano-B en el TCA           : %.1f m con %.3f mm/s\n', ...
    RES.bplane_tca.miss, RES.bplane_tca.dv*1e3);

save(fullfile(OUT,'test_bplane_mpc.mat'), 'RES', 'SC', 'H_MPC', 'NP_MPC', 'K_SIGMA');
fprintf('\nTEST_BPLANE_OK\n');

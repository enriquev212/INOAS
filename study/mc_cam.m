%MC_CAM Campana Monte Carlo sobre la geometria del encuentro.
%
% Convierte "tenemos un caso" en "tenemos un resultado". Todo lo medido hasta
% ahora sale de UNA geometria: un cruce a 88 deg, un angulo de plano-B y una
% distancia de fallo. Sin dispersion no se puede afirmar nada estadistico, y el
% titulo del paper lleva la palabra robust.
%
% Se muestrea:
%   dInc       angulo de cruce de planos -> fija |v_rel| = 2|v|sin(dInc/2)
%   bplane     orientacion del vector de fallo dentro del plano-B
%   miss0      distancia de maxima aproximacion sin maniobra
%   P_debris   covarianza del objeto: escala y orientacion
%
% Por cada geometria se construye la curva miss(dv) UNA vez y se invierte para
% cada nivel de incertidumbre de navegacion. Resolver desde cero para cada sigma
% costaria cuatro veces mas y daria lo mismo.
%
% Entradas (definir antes de llamar):
%   MC_CHUNK   indice del bloque (1..MC_NCHUNK), para repartir en procesos
%   MC_NCHUNK  numero de bloques
%   MC_N       muestras por bloque

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

if ~exist('MC_CHUNK','var');  MC_CHUNK = 1;  end
if ~exist('MC_NCHUNK','var'); MC_NCHUNK = 1; end
if ~exist('MC_N','var');      MC_N = 40;     end

STUDY_FORCE_MAIN = ~exist('MC_REPO','var');
if exist('MC_REPO','var'); REPO = MC_REPO; else
    REPO = REPO_ROOT; end
addpath(REPO); addpath(genpath(fullfile(REPO,'matlab')));
addpath(STUDY_DIR);
OUT = OUT_DIR;

MU = 3.986004418e14;
H_MPC = 30; NP_MPC = 60; TF = 12000; T_TCA = 10115;
% Los dos primeros valores son los MEDIDOS sobre el modelo completo: 4.8 m
% justo tras una correccion de GNSS y 32.1 m con el filtro saturado en
% propagacion. Ese es el rango en el que la arquitectura opera de verdad. El
% resto se mantiene para dibujar la tendencia mas alla de ese rango.
SIG_LIST = [4.8 12 32.1 40 100 250];   % radio 3D de navegacion en el TCA [m]
% Incertidumbre del OBJETO como eje propio, no como ruido. Con efemerides de
% catalogo (cientos de metros) domina el margen y la calidad de navegacion casi
% no importa; con efemerides frescas la relacion se invierte. Reportar un solo
% par de numeros esconde exactamente eso.
SIG_OBJ_LIST = [10 30 100 300];        % 1-sigma del objeto a lo largo de orbita [m]
K_SIGMA  = 3;
% El radio nominal es el MISMO que usa el controlador. Antes se usaba aqui
% R_HB = 5 m frente a dsafe0 = 150 m del MPC: el Monte Carlo invertia una ley
% de guiado distinta de la que se ejecuta, y sus medianas no eran comparables
% con ningun resultado del lazo cerrado.
D_SAFE0  = 150;
R_HB     = 5;
% Ambos signos: segun la geometria del encuentro, el impulso que abre la
% distancia es posigrado o retrogrado. Una rejilla solo positiva devuelve
% valores sin sentido en las geometrias de sensibilidad negativa.
DV_GRID  = [-160 -80 -40 -20 -10 -5 0 5 10 20 40 80 160]*1e-3;

setpref('inoas','skipBatchClear', true);
setpref('inoas','mpcTuneConfig', struct('h',H_MPC,'Np',NP_MPC));
simulationStopTime = TF; mpcQuiet = true;
evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));
T_orb = 2*pi/sqrt(MU/a^3);
T_BURN = T_TCA - T_orb;                  % una orbita de antelacion

rng(9000 + MC_CHUNK, 'twister');         % reproducible por bloque
odeo = odeset('RelTol',1e-9,'AbsTol',1e-8);

M = struct('dInc',{},'vrel',{},'bang',{},'miss0',{},'sens',{}, ...
           'sig',{},'sig_obj',{},'dtarget',{},'dv',{},'ok',{});

fprintf('bloque %d/%d, %d muestras\n', MC_CHUNK, MC_NCHUNK, MC_N);
tAll = tic;
for s = 1:MC_N
    % ---- muestrear la geometria -----------------------------------------
    dInc  = 20 + 140*rand;                    % 20..160 deg -> 2.5..14.2 km/s
    bang  = 360*rand;
    miss0 = 30 + 270*rand;                    % 30..300 m

    % Orientacion aleatoria, magnitud controlada por SIG_OBJ_LIST mas abajo.
    [Qr,~] = qr(randn(3));
    P_deb_shape = Qr * diag([1 0.4 0.4].^2) * Qr.';   % forma, sigma a lo largo = 1
    P_deb = (100^2) * P_deb_shape;                    % solo para construir la geometria

    SC = get_conjunction_scenario(h, x_ref_hist, t_ref, T_TCA, ...
        'dInc_deg', dInc, 'miss_m', miss0, 'bplane_deg', bang, ...
        'P_debris', P_deb, 'R_hb', R_HB);

    % ---- curva miss(dv) para esta geometria ------------------------------
    [~, iB] = min(abs(t_ref - T_BURN));
    x_b   = x_ref_hist(:, iB);
    x_d_b = SC.x_debris_hist(:, iB);
    tb    = t_ref(iB);
    v_hat = x_b(4:6)/norm(x_b(4:6));

    miss_curve = zeros(size(DV_GRID));
    for kdv = 1:numel(DV_GRID)
        xb = x_b;  xb(4:6) = xb(4:6) + DV_GRID(kdv)*v_hat;
        Tk = cam_find_tca(xb, x_d_b, tb, SC.t_tca, 40);
        miss_curve(kdv) = Tk.miss;
    end
    % Sensibilidad local alrededor de dv = 0, en metros de fallo por mm/s
    i0  = find(DV_GRID == 0, 1);
    DVP = DV_GRID(i0+1);            % paso de sonda para identificar la parabola
    sens = (miss_curve(i0+1) - miss_curve(i0-1)) / ((DV_GRID(i0+1)-DV_GRID(i0-1))*1e3);

    % ---- invertir la curva para cada nivel de incertidumbre --------------
    u_b = SC.d_nom_eci / norm(SC.d_nom_eci);
    for sg = SIG_LIST
      for so = SIG_OBJ_LIST
        % sg es un RADIO 3D (sqrt(traza(P))), que es lo que registra el filtro.
        % Construir diag([sg sg sg].^2) con el multiplica la varianza por tres.
        P_deb_k = (so^2) * P_deb_shape;
        P_comb  = (sg^2/3)*eye(3) + P_deb_k;
        sigma_b = sqrt(u_b.' * P_comb * u_b);
        dtar    = D_SAFE0 + K_SIGMA * sigma_b;

        if miss_curve(i0) >= dtar
            dv = 0;  ok = true;                      % no hace falta maniobrar
        else
            % La distancia de fallo al cuadrado es EXACTAMENTE cuadratica en dv,
            % asi que la raiz se obtiene en cerrado. Interpolar linealmente sobre
            % la rejilla, como se hacia antes, deja la cuerda por encima de una
            % funcion convexa y devuelve siempre una raiz por defecto: sesgo de un
            % solo signo que inflaba la razon entre regimenes un 47 %.
            Cq = miss_curve(i0)^2;
            Aq = (miss_curve(i0+1)^2 + miss_curve(i0-1)^2 - 2*Cq) / (2*DVP^2);
            Bq = (miss_curve(i0+1)^2 - miss_curve(i0-1)^2) / (2*DVP);
            dv = NaN;
            if Aq > 0
                disc = Bq^2 - 4*Aq*(Cq - dtar^2);
                if disc >= 0
                    rts = [(-Bq - sqrt(disc))/(2*Aq), (-Bq + sqrt(disc))/(2*Aq)];
                    vtx = -Bq/(2*Aq);
                    [~, ordr] = sort(abs(rts));
                    for cnd = rts(ordr)
                        if ~(vtx > min(0,cnd) && vtx < max(0,cnd)); dv = cnd; break; end
                    end
                    if isnan(dv); dv = rts(ordr(1)); end
                end
            end
            ok = isfinite(dv);
        end

        M(end+1) = struct('dInc',dInc,'vrel',SC.v_rel_mag,'bang',bang, ...
            'miss0',miss_curve(i0),'sens',sens,'sig',sg,'sig_obj',so, ...
            'dtarget',dtar,'dv',dv,'ok',ok); %#ok<SAGROW>
      end
    end

    if mod(s,10)==0
        fprintf('  %3d/%d  (%.1f s)\n', s, MC_N, toc(tAll));
    end
end

save(fullfile(OUT, sprintf('mc_cam_%02d.mat', MC_CHUNK)), 'M', 'SIG_LIST', ...
     'SIG_OBJ_LIST', 'K_SIGMA', 'D_SAFE0', 'DV_GRID', 'T_BURN', 'T_orb');
fprintf('bloque %d listo en %.1f s\n', MC_CHUNK, toc(tAll));
fprintf('MC_CHUNK_OK %d\n', MC_CHUNK);

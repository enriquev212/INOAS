%CAM_DEMO Demostrador: conjuncion realista sobre la orbita de INOAS.
%
% Objetivo: comprobar con numeros si la arquitectura del proyecto funciona con
% una conjuncion de verdad (cruce de planos, ~10 km/s) en lugar del encuentro
% co-orbital a 10 m/s que usa hoy, y cuantificar que cambia.
%
% No toca el repositorio.
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
OUT = OUT_DIR;
mu = 3.986004418e14;

%% ---------------- Orbita del satelite (la de INOAS) ---------------------
a = 7714.43e3; ecc = 0.000095; inc = 63.04; RAAN = 116.6; w = 90; nu = 131;
n_mm  = sqrt(mu/a^3);  T_orb = 2*pi/n_mm;
fprintf('Orbita INOAS: a = %.1f km, T = %.0f s (%.1f min), v = %.3f km/s\n', ...
    a/1e3, T_orb, T_orb/60, sqrt(mu/a)/1e3);

x0 = coe2rv(a, ecc, inc, RAAN, w, nu, mu);

%% ---------------- Escenario del encuentro -------------------------------
LEAD_ORB = 3;                       % TCA a 3 orbitas del instante inicial
t_tca_nom = LEAD_ORB * T_orb;
DINC   = 88.15;                     % cruce de planos [deg] -> ~10 km/s
MISS0  = 120;                       % distancia de maxima aproximacion sin maniobra [m]
BANG   = 25;                        % orientacion del fallo en el plano-B [deg]
R_HB   = 5;                         % radio de cuerpo duro combinado [m]

opt = odeset('RelTol',1e-11,'AbsTol',1e-9);
[~, X] = ode113(@(t,x) inoas_dyn(t,x,zeros(3,1)), [0 t_tca_nom], x0, opt);
x_c_tca = X(end,:).';

C = cam_build_conjunction(x_c_tca, DINC, MISS0, BANG);
fprintf('\nConjuncion construida:\n');
fprintf('  cruce de planos      = %.2f deg\n', C.dInc_deg);
fprintf('  |v_rel|              = %.3f km/s\n', C.v_rel_mag/1e3);
fprintf('  semieje sat / objeto = %.3f / %.3f km  (deben coincidir)\n', ...
    C.a_chaser/1e3, C.a_debris/1e3);
fprintf('  paso por esfera de 150 m = %.1f ms\n', 2*150/C.v_rel_mag*1e3);

% Retropropagar el objeto al instante inicial
[~, Xd] = ode113(@(t,x) inoas_dyn(t,x,zeros(3,1)), [t_tca_nom 0], C.x_d_tca, opt);
x_d0 = Xd(end,:).';

%% ---------------- Lo que ve la restriccion actual -----------------------
fprintf('\n--- Por que la restriccion actual no puede ver esto ---\n');
for h = [3 5 10 60]
    fprintf('  h = %2d s: el objeto recorre %8.1f km entre muestras; ', h, C.v_rel_mag*h/1e3);
    fprintf('P(alguna muestra dentro de la KOZ) ~ %.2e\n', min(1, (2*150/C.v_rel_mag)/h));
end

%% ---------------- Geometria sin maniobra --------------------------------
T0 = cam_find_tca(x0, x_d0, 0, t_tca_nom, 90);
fprintf('\n--- Sin maniobra ---\n');
fprintf('  TCA        = %.3f s (%.4f orbitas)\n', T0.t_tca, T0.t_tca/T_orb);
fprintf('  miss       = %.2f m\n', T0.miss);
fprintf('  |v_rel|    = %.3f km/s\n', T0.v_rel/1e3);

% Covarianzas en el TCA. La del satelite depende del ciclo de trabajo del GNSS:
% cuanto mas tiempo lleve apagado, mas ha crecido. La del objeto es la que
% publican los catalogos para un objeto pequeno en LEO.
P_deb = diag([150 60 60].^2);        % [m^2] elipsoide tipico de catalogo
sig_nav_list = [12 40 100 250];      % 1-sigma de navegacion [m]

fprintf('\n  %-12s %10s %10s %12s\n', 'sigma_nav', 'miss 2D', 'Mahal', 'Pc');
for s = sig_nav_list
    Bb = cam_bplane(T0, diag([s s s].^2), P_deb, R_HB);
    fprintf('  %8.0f m   %8.2f m %8.2f s %12.3e\n', s, Bb.miss2D, Bb.mahal, Bb.Pc);
end

%% ---------------- Maniobra: barrido de antelacion -----------------------
% Impulso tangencial (a lo largo de la velocidad). La deriva secular a lo largo
% de la orbita es lo que abre la distancia; por eso el delta-v necesario cae con
% la antelacion. Se aplica de verdad y se re-propaga: nada es analitico.
fprintf('\n--- Maniobra tangencial: delta-v necesario segun antelacion ---\n');
fprintf('  %-10s %12s %12s %12s %12s\n', 'antelacion', 'dv [mm/s]', 'miss [m]', 'Pc', 'quemado [s]');

leads   = [0.5 1 2 3] * T_orb;       % antelacion respecto al TCA
dv_test = 0.020;                     % impulso de prueba [m/s]
u_max_cs = 1.0/24;                   % propulsor 1 N sobre 24 kg [m/s^2]
sweep = struct('lead',{},'dv_req',{},'miss',{},'Pc',{});

TARGET_MISS = 500;                   % objetivo de separacion [m]
for kL = 1:numel(leads)
    L = leads(kL);
    t_burn = t_tca_nom - L;

    % La respuesta NO es lineal en delta-v: extrapolar desde un impulso de
    % prueba sobrestima. Se arranca con la estimacion lineal y se refina por
    % secante sobre la propagacion real hasta clavar el objetivo.
    [dv_req, misR, TR] = solveForMiss(x0, x_d0, t_burn, TARGET_MISS, ...
                                      T0.miss, dv_test, t_tca_nom, opt);
    BR = cam_bplane(TR, diag([40 40 40].^2), P_deb, R_HB);

    fprintf('  %6.2f orb  %12.3f %12.1f %12.3e %12.3f\n', ...
        L/T_orb, dv_req*1e3, misR, BR.Pc, dv_req/u_max_cs);

    sweep(end+1) = struct('lead',L,'dv_req',dv_req,'miss',misR,'Pc',BR.Pc); %#ok<SAGROW>
end

%% ---------------- Acoplamiento con la navegacion ------------------------
% El resultado que interesa al paper: con el GNSS apagado, la covarianza en el
% TCA crece, y con ella el fallo que hace falta para mantener Pc bajo umbral.
% Criterio de diseno: margen de K sigmas en el plano-B, que es el analogo
% directo de la KOZ inflada del proyecto (d_safe = d0 + k*sigma). Se usa este y
% no un umbral de Pc porque la probabilidad de colision NO es monotona en la
% incertidumbre: al crecer sigma con el fallo fijo, la densidad se reparte y Pc
% BAJA. Es la dilucion de probabilidad, y disenar contra un umbral de Pc a secas
% premia tener mala navegacion. Se reporta Pc al lado para que el efecto se vea.
K_SIG = 3;
fprintf('\n--- Margen de %d sigma en el plano-B, segun incertidumbre de navegacion ---\n', K_SIG);
fprintf('  %-10s %12s %11s %11s %13s %13s\n', ...
    'sigma_nav', 'miss req', 'dv [mm/s]', 'quemado', 'Pc sin manio.', 'Pc con manio.');

L = 1 * T_orb;  t_burn = t_tca_nom - L;
[mis1, ~] = missAfterBurn(x0, x_d0, t_burn, dv_test, t_tca_nom, opt);
slope = (mis1 - T0.miss) / dv_test;
fprintf('  (sensibilidad local en %g mm/s: %.1f m de fallo por cada mm/s, 1 orbita de antelacion)\n', ...
    dv_test*1e3, slope/1e3);

nav_sweep = struct('sig',{},'miss_req',{},'dv',{},'Pc0',{},'Pc1',{});
for s = sig_nav_list
    P_nav = diag([s s s].^2);
    B0 = cam_bplane(T0, P_nav, P_deb, R_HB);
    % mahal es lineal en |b| para direccion fija: m_req = K / sqrt(u' C^-1 u)
    u_dir = B0.b / norm(B0.b);
    m_req = K_SIG / sqrt(u_dir.' * (B0.C \ u_dir));
    [dv_req, ~, ~] = solveForMiss(x0, x_d0, t_burn, m_req, T0.miss, dv_test, ...
                                  t_tca_nom, opt);
    B1 = cam_bplane(setMiss(T0, m_req), P_nav, P_deb, R_HB);
    fprintf('  %7.0f m %12.1f m %11.3f %9.3f s %13.3e %13.3e\n', ...
        s, m_req, dv_req*1e3, dv_req/u_max_cs, B0.Pc, B1.Pc);
    nav_sweep(end+1) = struct('sig',s,'miss_req',m_req,'dv',dv_req, ...
        'Pc0',B0.Pc,'Pc1',B1.Pc); %#ok<SAGROW>
end

save(fullfile(OUT,'cam_demo.mat'), 'C', 'T0', 'sweep', 'nav_sweep', 'T_orb', ...
     'sig_nav_list', 'P_deb', 'R_HB', 'MISS0');
fprintf('\nCAM_DEMO_OK\n');

%% ======================= funciones auxiliares ===========================
function [mis, T] = missAfterBurn(x_c0, x_d0, t_burn, dv, t_tca_nom, opt)
    % OJO: los dos objetos tienen que estar referidos al MISMO instante antes de
    % llamar a cam_find_tca. Propagar solo el satelite hasta t_burn y pasar el
    % estado del objeto en t=0 deja al objeto atrasado t_burn segundos.
    if t_burn > 1e-9
        [~, Xb] = ode113(@(t,x) inoas_dyn(t,x,zeros(3,1)), [0 t_burn], x_c0, opt);
        xb = Xb(end,:).';
        [~, Xd] = ode113(@(t,x) inoas_dyn(t,x,zeros(3,1)), [0 t_burn], x_d0, opt);
        xd = Xd(end,:).';
    else
        xb = x_c0;  xd = x_d0;
    end
    vhat = xb(4:6)/norm(xb(4:6));
    xb(4:6) = xb(4:6) + dv*vhat;            % impulso tangencial
    T = cam_find_tca(xb, xd, t_burn, t_tca_nom, 120);
    mis = T.miss;
end

function [dv, mis, T] = solveForMiss(x_c0, x_d0, t_burn, target, miss0, dv_seed, ...
                                     t_tca_nom, opt)
%SOLVEFORMISS Impulso tangencial que produce la distancia de paso pedida.
%
%   Un impulso tangencial mueve el vector de fallo en el plano-B a lo largo de
%   UNA sola direccion, asi que el cuadrado de la distancia de paso es una
%   parabola exacta en dv:
%
%       m2(dv) = |b0 + dv*D|^2 = |D|^2 dv^2 + 2 (b0.D) dv + |b0|^2
%
%   Tres propagaciones la identifican y el impulso sale en forma cerrada. Antes
%   esto era una secante limitada a dv >= 0, que es la ley que el commit c2bb207
%   saco de plan_cam.m por estar mal de tres maneras:
%
%     - La rama posigrada no siempre es la barata. Aqui cuesta entre un 13 y un
%       38 % mas que la retrograda.
%     - La rama posigrada no es monotona: la distancia de paso primero BAJA, de
%       modo que una quemada parcial deja la nave mas cerca que no maniobrar.
%       La retrograda es monotona y por tanto tolerante a fallo.
%     - Decidir alcanzabilidad con el signo de una diferencia finita sobre una
%       funcion no monotona rechaza casos que si tienen solucion.
%
%   Se toma la raiz de menor modulo, descartando la rama cuyo vertice cae entre
%   cero y la propia raiz, que es justamente la que pasaria por debajo de miss0.
    p = abs(dv_seed);
    [mp, ~] = missAfterBurn(x_c0, x_d0, t_burn,  p, t_tca_nom, opt);
    [mm, ~] = missAfterBurn(x_c0, x_d0, t_burn, -p, t_tca_nom, opt);
    m0 = miss0;

    C = m0^2;
    A = (mp^2 + mm^2 - 2*C) / (2*p^2);
    B = (mp^2 - mm^2) / (2*p);

    dv = 0;
    if A > 0
        disc = B^2 - 4*A*(C - target^2);
        if disc >= 0
            r = (-B + [1 -1]*sqrt(disc)) / (2*A);
            vertex = -B / (2*A);            % donde la parabola toca su minimo
            % Descartar la rama que pasa por el vertice antes de llegar al
            % objetivo: ahi la distancia de paso baja por debajo de miss0.
            keep = ~((r > 0 & vertex > 0 & vertex < r) | ...
                     (r < 0 & vertex < 0 & vertex > r));
            r = r(keep);
            if ~isempty(r)
                [~, i] = min(abs(r));
                dv = r(i);
            end
        end
    end

    [mis, T] = missAfterBurn(x_c0, x_d0, t_burn, dv, t_tca_nom, opt);
end

function T2 = setMiss(T, m)
    T2 = T;
    T2.dr = T.dr * (m / norm(T.dr));
end

function x = bisect(f, lo, hi)
    flo = f(lo);
    if flo <= 0; x = lo; return; end
    for i = 1:80
        mid = 0.5*(lo+hi);
        if f(mid) > 0; lo = mid; else; hi = mid; end
        if hi-lo < 1e-3; break; end
    end
    x = 0.5*(lo+hi);
end

function x = coe2rv(a, e, i, RAAN, w, nu, mu)
    i = deg2rad(i); RAAN = deg2rad(RAAN); w = deg2rad(w); nu = deg2rad(nu);
    p = a*(1-e^2);  r = p/(1+e*cos(nu));
    r_pf = [r*cos(nu); r*sin(nu); 0];
    v_pf = sqrt(mu/p)*[-sin(nu); e+cos(nu); 0];
    Rz_R = [cos(RAAN) -sin(RAAN) 0; sin(RAAN) cos(RAAN) 0; 0 0 1];
    Rx_i = [1 0 0; 0 cos(i) -sin(i); 0 sin(i) cos(i)];
    Rz_w = [cos(w) -sin(w) 0; sin(w) cos(w) 0; 0 0 1];
    Q = Rz_R*Rx_i*Rz_w;
    x = [Q*r_pf; Q*v_pf];
end

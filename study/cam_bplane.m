function B = cam_bplane(T, P_nav, P_debris, R_hb)
%CAM_BPLANE Geometria en el plano-B y probabilidad de colision.
%
%   El plano-B es el plano perpendicular a la velocidad relativa en el TCA. Es
%   donde la comunidad de evitacion de colision evalua el riesgo, y por dos
%   razones que importan aqui:
%
%     1. La restriccion deja de depender del muestreo temporal. En vez de exigir
%        ||r_rel|| >= d_safe en cada paso de la rejilla (imposible a 10 km/s), se
%        exige que la distancia de maxima aproximacion PROYECTADA supere el
%        umbral. Es una unica restriccion, evaluada donde de verdad importa.
%
%     2. La incertidumbre del objeto entra de forma natural: en el plano-B la
%        covarianza relevante es la COMBINADA de los dos objetos, sumada y
%        proyectada a 2D. En la formulacion actual del proyecto la covarianza del
%        objeto simplemente no existe.
%
%   Entradas:
%     T          salida de cam_find_tca
%     P_nav      covarianza de posicion del satelite en el TCA [3x3]
%     P_debris   covarianza de posicion del objeto en el TCA  [3x3]
%     R_hb       radio de cuerpo duro combinado [m]

    u_hat = T.dv / norm(T.dv);

    % Base del plano-B
    tmp = cross(T.x_d(4:6), T.x_c(4:6));
    if norm(tmp) < 1e-9; tmp = cross(u_hat, T.x_c(1:3)); end
    xi_hat = tmp - (tmp.'*u_hat)*u_hat;
    xi_hat = xi_hat / norm(xi_hat);
    zeta_hat = cross(u_hat, xi_hat);

    J = [xi_hat.'; zeta_hat.'];          % proyeccion 3D -> 2D

    b2 = J * T.dr;                        % vector de fallo proyectado
    C2 = J * (P_nav + P_debris) * J.';    % covarianza combinada proyectada
    C2 = 0.5*(C2 + C2.');

    B = struct();
    B.b        = b2;
    B.miss2D   = norm(b2);
    B.C        = C2;
    B.sigma    = sqrt(eig(C2));
    B.u_hat    = u_hat;
    B.xi_hat   = xi_hat;
    B.zeta_hat = zeta_hat;
    B.Pc       = collisionProbability(b2, C2, R_hb);
    % Distancia de Mahalanobis: cuantas sigmas de margen hay
    B.mahal    = sqrt(b2.' * (C2 \ b2));
end

function Pc = collisionProbability(b, C, R)
%COLLISIONPROBABILITY Integral 2D de la gaussiana sobre el disco de radio R.
%   Formulacion de Foster: se integra la densidad de la posicion relativa sobre
%   el area del cuerpo duro, en coordenadas polares centradas en el objeto.
    if R <= 0; Pc = 0; return; end
    detC = det(C);
    if detC <= 0; Pc = NaN; return; end
    Ci = inv(C);
    k  = 1/(2*pi*sqrt(detC));

    nr = 200; nth = 360;
    rr  = linspace(0, R, nr);
    tth = linspace(0, 2*pi, nth);
    [RR, TT] = ndgrid(rr, tth);
    X = RR.*cos(TT) - b(1);
    Y = RR.*sin(TT) - b(2);
    E = Ci(1,1)*X.^2 + 2*Ci(1,2)*X.*Y + Ci(2,2)*Y.^2;
    F = k * exp(-0.5*E) .* RR;            % jacobiano polar

    Pc = trapz(rr, trapz(tth, F, 2));
end

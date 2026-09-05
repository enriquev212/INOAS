function T = cam_find_tca(x_c0, x_d0, t0, t_guess, tspan_half)
%CAM_FIND_TCA Instante y geometria de maxima aproximacion, por refinamiento
%continuo.
%
%   A 10 km/s de velocidad relativa el paso por una esfera de 150 m dura 30 ms.
%   Buscar el minimo sobre la rejilla del horizonte del MPC (pasos de 3 a 60 s)
%   es imposible: el objeto recorre entre 30 y 600 km entre muestras. El TCA hay
%   que resolverlo de forma continua.
%
%   Se propagan ambos objetos con dinamica de dos cuerpos mas J2 y se minimiza
%   la distancia con fminbnd sobre una ventana alrededor de la estimacion.
%
%   Entradas:
%     x_c0, x_d0  estados iniciales [6x1] en t0
%     t_guess     estimacion del TCA [s]
%     tspan_half  semiventana de busqueda alrededor de t_guess [s]

    if nargin < 5 || isempty(tspan_half); tspan_half = 60; end
    opt = odeset('RelTol',1e-11,'AbsTol',1e-9);

    % Propagar ambos hasta el inicio de la ventana de busqueda
    tA = t_guess - tspan_half;
    xc_A = propag(x_c0, [t0 tA], opt);
    xd_A = propag(x_d0, [t0 tA], opt);

    dist = @(dt) norm(sub(propag(xc_A, [tA tA+dt], opt), ...
                          propag(xd_A, [tA tA+dt], opt)));

    fopt = optimset('TolX', 1e-9);
    [dt_star, dmin] = fminbnd(dist, 0, 2*tspan_half, fopt);

    t_star = tA + dt_star;
    xc = propag(xc_A, [tA t_star], opt);
    xd = propag(xd_A, [tA t_star], opt);

    dr = xd(1:3) - xc(1:3);
    dv = xd(4:6) - xc(4:6);

    T = struct();
    T.t_tca   = t_star;
    T.miss    = dmin;
    T.dr      = dr;
    T.dv      = dv;
    T.v_rel   = norm(dv);
    T.x_c     = xc;
    T.x_d     = xd;
end

function y = propag(x0, tspan, opt)
    if abs(tspan(2)-tspan(1)) < 1e-12; y = x0; return; end
    [~, X] = ode113(@(t,x) inoas_dyn(t,x,zeros(3,1)), tspan, x0, opt);
    y = X(end,:).';
end

function d = sub(a, b)
    d = a(1:3) - b(1:3);
end

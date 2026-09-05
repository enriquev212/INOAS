function dx = inoas_dyn(~, x, u)
%INOAS_DYN Planta verdadera: dos cuerpos + J2, con aceleracion de control ZOH.
    mu = 3.986004418e14; Re = 6378137.0; J2 = 1.08262668e-3;
    r = x(1:3); rn = norm(r);
    a2b = -mu*r/rn^3;
    zr = r(3)/rn; kk = 1.5*J2*mu*Re^2/rn^4;
    aJ2 = -kk*[ (1-5*zr^2)*r(1)/rn; (1-5*zr^2)*r(2)/rn; (3-5*zr^2)*r(3)/rn ];
    dx = [x(4:6); a2b + aJ2 + u(:)];
end

function get_nominal_trajectory(Ts, Nsteps, filename, varargin)

    if nargin < 3
        filename = inoas_data_path("referenceTrajectory.mat");
    end

    p = inputParser;
    p.addParameter("startDateJulian", juliandate(datetime(2024,1,11)));
    p.addParameter("a", 7714.43 * 1000);
    p.addParameter("ecc", 0.000095);
    p.addParameter("incl", 63.04);
    p.addParameter("RAAN", 116.6);
    p.addParameter("argp", 90);
    p.addParameter("nu", 131);
    p.addParameter("PropModel", "two-body-keplerian");
    p.parse(varargin{:});
    opts = p.Results;

    t0 = datetime(opts.startDateJulian, "ConvertFrom", "juliandate");

    t_ref = (0:Nsteps-1)' * Ts;
    t_vec = t0 + seconds(t_ref);

    a    = opts.a;
    ecc  = opts.ecc;
    incl = opts.incl;
    RAAN = opts.RAAN;
    argp = opts.argp;
    nu   = opts.nu;

    propModel = string(opts.PropModel);
    if strcmpi(propModel, "j2-rk4")
        [pos, vel] = propagateJ2Reference(t_ref, a, ecc, incl, RAAN, argp, nu);
    else
        [pos, vel] = propagateOrbit(t_vec, a, ecc, incl, RAAN, argp, nu, ...
                                    'PropModel','two-body-keplerian');
    end

    x_ref_hist = [pos; vel];   % [6 x Nsteps]

    r_p_full = reshape(x_ref_hist, [], 1);

    save(filename,'r_p_full','x_ref_hist','t_ref');

end

function [pos, vel] = propagateJ2Reference(t_ref, a, ecc, inclDeg, raanDeg, argpDeg, nuDeg)
    x = coeToEciState(a, ecc, deg2rad(inclDeg), deg2rad(raanDeg), ...
                      deg2rad(argpDeg), deg2rad(nuDeg));

    pos = zeros(3, numel(t_ref));
    vel = zeros(3, numel(t_ref));
    pos(:,1) = x(1:3);
    vel(:,1) = x(4:6);

    for k = 2:numel(t_ref)
        dt = t_ref(k) - t_ref(k-1);
        x = rk4StepJ2(x, dt);
        pos(:,k) = x(1:3);
        vel(:,k) = x(4:6);
    end
end

function x = coeToEciState(a, ecc, incl, raan, argp, nu)
    mu = 3.986004418e14;
    p = a * (1 - ecc^2);

    rPqw = p / (1 + ecc*cos(nu)) * [cos(nu); sin(nu); 0];
    vPqw = sqrt(mu / p) * [-sin(nu); ecc + cos(nu); 0];

    cO = cos(raan); sO = sin(raan);
    ci = cos(incl); si = sin(incl);
    cw = cos(argp); sw = sin(argp);

    R3O = [ cO -sO 0; sO cO 0; 0 0 1];
    R1i = [ 1 0 0; 0 ci -si; 0 si ci];
    R3w = [ cw -sw 0; sw cw 0; 0 0 1];
    Q = R3O * R1i * R3w;

    x = [Q*rPqw; Q*vPqw];
end

function xNext = rk4StepJ2(x, dt)
    k1 = derivativeJ2(x);
    k2 = derivativeJ2(x + 0.5*dt*k1);
    k3 = derivativeJ2(x + 0.5*dt*k2);
    k4 = derivativeJ2(x + dt*k3);
    xNext = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function xDot = derivativeJ2(x)
    r = x(1:3);
    v = x(4:6);

    mu = 3.986004418e14;
    Re = 6378137.0;
    J2 = 1.08262668e-3;

    rn = norm(r);
    xE = r(1); yE = r(2); zE = r(3);
    z2r2 = (zE/rn)^2;

    aCentral = -mu * r / rn^3;
    factor = 1.5 * J2 * mu * Re^2 / rn^5;
    aJ2 = factor * [ ...
        xE * (5*z2r2 - 1); ...
        yE * (5*z2r2 - 1); ...
        zE * (5*z2r2 - 3)];

    xDot = [v; aCentral + aJ2];
end

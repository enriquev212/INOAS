function get_nominal_trajectory(Ts, Nsteps, filename, varargin)

    if nargin < 3
        filename = "referenceTrajectory.mat";
    end

    p = inputParser;
    p.addParameter("startDateJulian", juliandate(datetime(2024,1,11)));
    p.addParameter("a", 7714.43 * 1000);
    p.addParameter("ecc", 0.000095);
    p.addParameter("incl", 63.04);
    p.addParameter("RAAN", 116.6);
    p.addParameter("argp", 90);
    p.addParameter("nu", 131);
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

    [pos, vel] = propagateOrbit(t_vec, a, ecc, incl, RAAN, argp, nu, ...
                                'PropModel','two-body-keplerian');

    x_ref_hist = [pos; vel];   % [6 x Nsteps]

    r_p_full = reshape(x_ref_hist, [], 1);

    save(filename,'r_p_full','x_ref_hist','t_ref');

end

function [r_p_full, x_ref_hist, t_ref, meta] = get_nominal_trajectory_from_gnss(Ts, Nsteps, filename)

    if nargin < 3
        filename = inoas_data_path("referenceTrajectory.mat");
    end

    %% ============================================================
    %  CONFIGURATION
    % ============================================================

    gnss_file = inoas_data_file("cov_perturb_POS_s6a_Y24D011_fixed.dat");

    omega_E = 7.2921159e-5;      % [rad/s] Earth rotation rate
    mu      = 3.986004418e14;    % [m^3/s^2]

    % Mantengo el suavizado de posicion como lo tenias.
    % Si ves picos raros, prueba smooth_win = 1.
    smooth_win = 7;

    % Saltar primeras muestras si quieres evitar borde inicial raro.
    % Con 1 no se salta nada.
    idx_start = 1;

    %% ============================================================
    %  1) READ TEXT .DAT GNSS FILE
    % ============================================================

    if ~isfile(gnss_file)
        error("No encuentro el archivo GNSS: %s", gnss_file);
    end

    data = readmatrix(gnss_file, "CommentStyle", "#");

    if isempty(data)
        error("El archivo GNSS se ha leido vacio.");
    end

    if size(data,2) < 14
        error("El archivo GNSS debe tener al menos 14 columnas.");
    end

    %% ============================================================
    %  2) FILTER VALID SOLUTIONS
    % ============================================================

    data = data(data(:,7) == 1, :);

    if isempty(data)
        error("No quedan muestras GNSS validas tras filtrar SOL == 1.");
    end

    %% ============================================================
    %  3) EXTRACT COLUMNS
    % ============================================================

    t_raw   = data(:,1);     % SOD [s]
    lon_deg = data(:,2);     % longitude [deg]
    lat_deg = data(:,3);     % latitude [deg]
    alt     = data(:,4);     % altitude [m]

    %% ============================================================
    %  4) SKIP INITIAL SAMPLES IF REQUIRED
    % ============================================================

    if idx_start > 1
        if numel(t_raw) <= idx_start
            error("No hay suficientes muestras GNSS para aplicar idx_start.");
        end

        t_raw   = t_raw(idx_start:end);
        lon_deg = lon_deg(idx_start:end);
        lat_deg = lat_deg(idx_start:end);
        alt     = alt(idx_start:end);
    end

    %% ============================================================
    %  5) NORMALIZE TIME
    % ============================================================

    t_gnss = t_raw - t_raw(1);

    %% ============================================================
    %  6) LLA -> ECEF POSITION
    % ============================================================

    phi    = deg2rad(lat_deg);
    lambda = deg2rad(lon_deg);
    h_alt  = alt;

    a_wgs = 6378137.0;
    e_wgs = 8.1819190842622e-2;

    N = a_wgs ./ sqrt(1 - e_wgs^2 .* sin(phi).^2);

    x_ecef = (N + h_alt) .* cos(phi) .* cos(lambda);
    y_ecef = (N + h_alt) .* cos(phi) .* sin(lambda);
    z_ecef = (N .* (1 - e_wgs^2) + h_alt) .* sin(phi);

    pos_ecef_raw = [x_ecef.'; y_ecef.'; z_ecef.'];

    %% ============================================================
    %  7) ECEF VELOCITY FROM RAW ECEF POSITION
    % ============================================================
    % Esta velocidad esta en el frame rotante ECEF.
    % Luego se convierte a velocidad inercial ECI con:
    % v_eci = R * (v_ecef + omega x r_ecef)

    vx_ecef_raw = gradient(x_ecef, t_gnss);
    vy_ecef_raw = gradient(y_ecef, t_gnss);
    vz_ecef_raw = gradient(z_ecef, t_gnss);

    % Evitar artefactos de borde
    vx_ecef_raw(1)   = vx_ecef_raw(2);
    vy_ecef_raw(1)   = vy_ecef_raw(2);
    vz_ecef_raw(1)   = vz_ecef_raw(2);

    vx_ecef_raw(end) = vx_ecef_raw(end-1);
    vy_ecef_raw(end) = vy_ecef_raw(end-1);
    vz_ecef_raw(end) = vz_ecef_raw(end-1);

    vel_ecef_raw = [vx_ecef_raw.'; vy_ecef_raw.'; vz_ecef_raw.'];

    %% ============================================================
    %  8) MPC REFERENCE TIME VECTOR
    % ============================================================

    t_ref = (0:Nsteps-1)' * Ts;

    if t_ref(end) > t_gnss(end)
        error("La referencia pide %.3f s, pero el GNSS solo llega hasta %.3f s. Reduce tf/Np/h o usa un GNSS mas largo.", ...
              t_ref(end), t_gnss(end));
    end

    t_query = t_ref;

    %% ============================================================
    %  9) INTERPOLATE ECEF POSITION AND VELOCITY TO MPC GRID
    % ============================================================

    pos_ecef = zeros(3, Nsteps);
    vel_ecef = zeros(3, Nsteps);

    for k = 1:3
        pos_ecef(k,:) = interp1(t_gnss, pos_ecef_raw(k,:), t_query, "linear").';
        vel_ecef(k,:) = interp1(t_gnss, vel_ecef_raw(k,:), t_query, "linear").';
    end

    %% ============================================================
    %  10) ECEF -> ECI POSITION AND VELOCITY
    % ============================================================
    % ECI is aligned with ECEF at t = 0.
    %
    % r_eci = R * r_ecef
    % v_eci = R * (v_ecef + omega_E x r_ecef)

    pos_eci_raw = zeros(3, Nsteps);
    vel_eci_raw = zeros(3, Nsteps);

    omega_vec = [0; 0; omega_E];

    for i = 1:Nsteps
        th = omega_E * t_ref(i);

        R_ecef_to_eci = [ cos(th), -sin(th), 0;
                          sin(th),  cos(th), 0;
                          0,        0,       1 ];

        r_ecef_i = pos_ecef(:,i);
        v_ecef_i = vel_ecef(:,i);

        pos_eci_raw(:,i) = R_ecef_to_eci * r_ecef_i;
        vel_eci_raw(:,i) = R_ecef_to_eci * (v_ecef_i + cross(omega_vec, r_ecef_i));
    end

    %% ============================================================
    %  11) LIGHT SMOOTHING OF ECI POSITION
    % ============================================================
    % The MPC consumes position and velocity as one state. Recompute the
    % velocity from the final ECI position used below so both components are
    % kinematically consistent on the MPC grid.

    pos_eci = pos_eci_raw;

    win = smooth_win;

    if Nsteps < win
        win = Nsteps;
    end

    if mod(win,2) == 0
        win = win - 1;
    end

    if win >= 5
        halfwin = floor(win/2);

        for k = 1:3
            temp = smoothdata(pos_eci_raw(k,:), "sgolay", win);

            % Preserve edges to avoid boundary artefacts
            temp(1:halfwin) = pos_eci_raw(k,1:halfwin);
            temp(end-halfwin+1:end) = pos_eci_raw(k,end-halfwin+1:end);

            pos_eci(k,:) = temp;
        end
    end

    vel_eci = zeros(3, Nsteps);
    dt_ref = mean(diff(t_ref));

    for k = 1:3
        vel_eci(k,:) = gradient(pos_eci(k,:), dt_ref);
    end

    %% ============================================================
    %  12) FINAL REFERENCE
    % ============================================================

    x_ref_hist = [pos_eci; vel_eci];      % [6 x Nsteps]
    r_p_full   = reshape(x_ref_hist, [], 1);

    %% ============================================================
    %  13) SAVE .MAT OUTPUT
    % ============================================================

    meta = struct();
    meta.source = "GNSS text dat";
    meta.gnss_file = gnss_file;
    meta.frame = "ECI";
    meta.smooth_win = win;
    meta.interpolation = "linear";
    meta.velocity_source = "gradient of final ECI position";
    meta.idx_start = idx_start;
    meta.t_gnss_end = t_gnss(end);
    meta.t_ref_end = t_ref(end);
    meta.mu = mu;
    meta.omega_E = omega_E;

    save(filename, "r_p_full", "x_ref_hist", "t_ref", "meta");

    %% ============================================================
    %  14) CHECKS
    % ============================================================

    fprintf("\nGNSS reference generated from text .dat in ECI.\n");
    fprintf("Archivo GNSS usado: %s\n", gnss_file);
    fprintf("Archivo guardado: %s\n", filename);
    fprintf("interpolation usado: linear\n");
    fprintf("velocity source: gradient of final ECI position\n");
    fprintf("idx_start usado: %d\n", idx_start);
    fprintf("smooth_win usado: %d\n", win);
    fprintf("Tamaño x_ref_hist: [%d x %d]\n", size(x_ref_hist,1), size(x_ref_hist,2));
    fprintf("Tamaño r_p_full:   [%d x %d]\n", size(r_p_full,1), size(r_p_full,2));
    fprintf("t_ref(end)  = %.3f s\n", t_ref(end));
    fprintf("t_gnss(end) = %.3f s\n", t_gnss(end));
    fprintf("norm r0 = %.6f km\n", norm(x_ref_hist(1:3,1))/1000);
    fprintf("norm v0 = %.6f km/s\n", norm(x_ref_hist(4:6,1))/1000);

    speed = vecnorm(x_ref_hist(4:6,:), 2, 1);
    radius = vecnorm(x_ref_hist(1:3,:), 2, 1);

    fprintf("\nCHECK SPEED/RADIUS RANGE\n");
    fprintf("speed min   = %.6f km/s\n", min(speed)/1000);
    fprintf("speed max   = %.6f km/s\n", max(speed)/1000);
    fprintf("speed range = %.6f m/s\n", max(speed)-min(speed));
    fprintf("radius min  = %.6f km\n", min(radius)/1000);
    fprintf("radius max  = %.6f km\n", max(radius)/1000);
    fprintf("radius range= %.6f m\n\n", max(radius)-min(radius));

    dt = t_ref(2) - t_ref(1);
    v_fd_01 = (x_ref_hist(1:3,2) - x_ref_hist(1:3,1)) / dt;

    fprintf("CHECK POSITION-VELOCITY CONSISTENCY\n");
    fprintf("|v0 - v_fd_01| = %.6f m/s\n", norm(x_ref_hist(4:6,1) - v_fd_01));
    fprintf("Nota: v_ref se recalcula desde la posicion ECI final usada por el MPC.\n\n");

    %% ============================================================
    %  15) SIMPLE PLOTS
    % ============================================================

    figure("Name", "GNSS Reference from DAT");

    subplot(2,1,1)
    plot3(x_ref_hist(1,:), x_ref_hist(2,:), x_ref_hist(3,:), ".", "MarkerSize", 8);
    grid on;
    axis equal;
    title("Referencia GNSS en ECI");
    xlabel("x ECI [m]");
    ylabel("y ECI [m]");
    zlabel("z ECI [m]");

    subplot(2,1,2)
    plot(t_ref, vecnorm(x_ref_hist(4:6,:),2,1), "LineWidth", 1.2);
    grid on;
    title("Norma velocidad referencia GNSS en ECI");
    xlabel("t [s]");
    ylabel("|v| [m/s]");

end

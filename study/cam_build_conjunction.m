function C = cam_build_conjunction(x_c_tca, dInc_deg, miss_m, bplane_angle_deg)
%CAM_BUILD_CONJUNCTION Construye una conjuncion realista sobre la orbita dada.
%
%   La trayectoria actual del proyecto construye el objeto como un offset en
%   LVLH con velocidad relativa de 10 m/s, lo que implica una diferencia de
%   inclinacion de 0.08 deg: un objeto practicamente co-orbital, no una
%   conjuncion. Aqui se construye el objeto sobre una ORBITA PROPIA que cruza
%   la del satelite.
%
%   Metodo: se rota el vector velocidad del satelite alrededor de su propia
%   direccion radial un angulo dInc. Como la rotacion preserva |v| y el angulo
%   entre r y v, el objeto queda en una orbita con el MISMO semieje mayor y la
%   misma excentricidad, pero en otro plano. Es una conjuncion de cruce de
%   planos genuina, y la velocidad relativa sale sola:
%
%       |v_rel| = 2 |v| sin(dInc/2)
%
%   Entradas:
%     x_c_tca          estado del satelite en el TCA [6x1], ECI [m, m/s]
%     dInc_deg         angulo de cruce de planos [deg]
%     miss_m           distancia de maxima aproximacion sin maniobra [m]
%     bplane_angle_deg orientacion del vector de fallo dentro del plano-B [deg]
%
%   Salida: struct con el estado del objeto en el TCA y la geometria del
%   encuentro.

    r_c = x_c_tca(1:3);  v_c = x_c_tca(4:6);

    % --- orbita del objeto: rotar v_c alrededor de r_hat ---------------------
    r_hat = r_c / norm(r_c);
    th = deg2rad(dInc_deg);
    K = [    0      -r_hat(3)  r_hat(2);
          r_hat(3)      0     -r_hat(1);
         -r_hat(2)  r_hat(1)      0    ];
    R = eye(3) + sin(th)*K + (1-cos(th))*(K*K);     % Rodrigues
    v_d = R * v_c;

    v_rel = v_d - v_c;
    u_hat = v_rel / norm(v_rel);                     % eje del encuentro

    % --- vector de fallo, contenido en el plano-B (perpendicular a v_rel) ----
    % Base del plano-B: xi a lo largo de la normal comun, zeta completa el trio.
    tmp = cross(v_d, v_c);
    if norm(tmp) < 1e-9; tmp = cross(u_hat, r_hat); end
    xi_hat = tmp / norm(tmp);
    xi_hat = xi_hat - (xi_hat.'*u_hat)*u_hat;        % ortogonalizar
    xi_hat = xi_hat / norm(xi_hat);
    zeta_hat = cross(u_hat, xi_hat);

    phi = deg2rad(bplane_angle_deg);
    b_vec = miss_m * (cos(phi)*xi_hat + sin(phi)*zeta_hat);

    r_d = r_c + b_vec;

    C = struct();
    C.x_d_tca   = [r_d; v_d];
    C.v_rel     = v_rel;
    C.v_rel_mag = norm(v_rel);
    C.u_hat     = u_hat;
    C.xi_hat    = xi_hat;
    C.zeta_hat  = zeta_hat;
    C.b_vec     = b_vec;
    C.miss      = norm(b_vec);
    C.dInc_deg  = dInc_deg;
    % Comprobacion: misma energia => mismo semieje mayor que el satelite
    mu = 3.986004418e14;
    C.a_chaser  = 1/(2/norm(r_c) - norm(v_c)^2/mu);
    C.a_debris  = 1/(2/norm(r_d) - norm(v_d)^2/mu);
end

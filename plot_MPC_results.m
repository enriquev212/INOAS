%% plot_MPC_results_clean.m
% Postproceso esencial de la simulación MPC orbital

close all;

%% ============================================================
% 1. CARGA DE DATOS
% ============================================================

%% Referencia
x_ref = ref_ts_x.Data(:);
y_ref = ref_ts_y.Data(:);
z_ref = ref_ts_z.Data(:);
t_ref = time_ref(:);

vx_ref = ref_ts_vx.Data(:);
vy_ref = ref_ts_vy.Data(:);
vz_ref = ref_ts_vz.Data(:);

%% Estimado
est_log = out.logsout.get("Estimated_Pos_x").Values;
t_est = est_log.Time;
x_est_data = squeeze(est_log.Data);

x_est = x_est_data(:,1);
y_est = x_est_data(:,2);
z_est = x_est_data(:,3);

%% Real
real_log = out.logsout.get("X_perfect_sensor").Values;
t_real = real_log.Time;
x_real_data = squeeze(real_log.Data);

x_real = x_real_data(:,1);
y_real = x_real_data(:,2);
z_real = x_real_data(:,3);

%% Control MPC
u_log = codex_get_logsout_signal(out.logsout, {"u_MPC", "u_discret"});
t_u = u_log.Time;
u_MPC = squeeze(u_log.Data);

if size(u_MPC,2) ~= 3 && size(u_MPC,1) == 3
    u_MPC = u_MPC.';
end

has_dsafe_log = exist("codex_dsafe_log_time", "var") && ...
                exist("codex_dsafe_log_first", "var") && ...
                exist("codex_dsafe_log_max", "var");

if has_dsafe_log
    t_dsafe = codex_dsafe_log_time(:);
    dsafe_first = codex_dsafe_log_first(:);
    dsafe_horizon_max = codex_dsafe_log_max(:);
else
    t_dsafe = [];
    dsafe_first = [];
    dsafe_horizon_max = [];
end

%% Interpolación de referencia sobre tiempo real
x_ref_i  = interp1(t_ref, x_ref,  t_real, "pchip", "extrap");
y_ref_i  = interp1(t_ref, y_ref,  t_real, "pchip", "extrap");
z_ref_i  = interp1(t_ref, z_ref,  t_real, "pchip", "extrap");

vx_ref_i = interp1(t_ref, vx_ref, t_real, "pchip", "extrap");
vy_ref_i = interp1(t_ref, vy_ref, t_real, "pchip", "extrap");
vz_ref_i = interp1(t_ref, vz_ref, t_real, "pchip", "extrap");

has_debris_traj = exist("x_debris_hist", "var") && ~isempty(x_debris_hist);

if has_debris_traj
    if size(x_debris_hist,1) ~= 6 && size(x_debris_hist,2) == 6
        x_debris_hist = x_debris_hist.';
    end

    t_debris_hist = t_ref(1:size(x_debris_hist,2));
    x_debris = x_debris_hist(1,:).';
    y_debris = x_debris_hist(2,:).';
    z_debris = x_debris_hist(3,:).';

    x_debris_i = interp1(t_debris_hist, x_debris, t_real, "pchip", "extrap");
    y_debris_i = interp1(t_debris_hist, y_debris, t_real, "pchip", "extrap");
    z_debris_i = interp1(t_debris_hist, z_debris, t_real, "pchip", "extrap");

    if exist("t_debris", "var")
        [~, debris_marker_idx] = min(abs(t_debris_hist - t_debris));
    else
        debris_marker_idx = 1;
    end

    rk_debris_plot = x_debris_hist(1:3, debris_marker_idx);
else
    t_debris_hist = [];
    x_debris = [];
    y_debris = [];
    z_debris = [];
    x_debris_i = [];
    y_debris_i = [];
    z_debris_i = [];

    if exist("rk_debris", "var")
        rk_debris_plot = rk_debris(:);
    else
        rk_debris_plot = [];
    end
end

%% ============================================================
% 2. TRAYECTORIA 3D
% ============================================================

figure;
hold on; grid on; axis equal;

plot3(x_ref,  y_ref,  z_ref,  "k--", "LineWidth", 2);
plot3(x_real, y_real, z_real, "g",   "LineWidth", 1.4);
plot3(x_est,  y_est,  z_est,  "b",   "LineWidth", 1.2);

if has_debris_traj
    plot3(x_debris, y_debris, z_debris, "r-", "LineWidth", 1.2);
    plot3(rk_debris_plot(1), rk_debris_plot(2), rk_debris_plot(3), "ro", ...
        "MarkerSize", 8, "MarkerFaceColor", "r");
    legend("Referencia", "Real", "Estimado", "Trayectoria debris", "Debris en encuentro");
elseif exist("rk_debris","var")
    plot3(rk_debris_plot(1), rk_debris_plot(2), rk_debris_plot(3), "ro", ...
        "MarkerSize", 8, "MarkerFaceColor", "r");

    if exist("dsafe0","var") && dsafe0 > 0
        [Xs,Ys,Zs] = sphere(40);
        surf(rk_debris_plot(1) + dsafe0*Xs, ...
             rk_debris_plot(2) + dsafe0*Ys, ...
             rk_debris_plot(3) + dsafe0*Zs, ...
             "FaceAlpha", 0.15, ...
             "EdgeColor", "none");
        legend("Referencia", "Real", "Estimado", "Debris", "Zona segura");
    else
        legend("Referencia", "Real", "Estimado", "Debris");
    end
else
    legend("Referencia", "Real", "Estimado");
end

xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Trayectoria orbital");
view(3);

%% ============================================================
% 3. POSICIÓN X/Y/Z
% ============================================================

figure;

subplot(3,1,1);
plot(t_real, x_real, "g", "LineWidth", 1.3); hold on;
plot(t_est,  x_est,  "b", "LineWidth", 1.2);
plot(t_real, x_ref_i, "k--", "LineWidth", 1.4);
grid on;
ylabel("x [m]");
title("Posición X");
legend("Real","Estimado","Referencia");

subplot(3,1,2);
plot(t_real, y_real, "g", "LineWidth", 1.3); hold on;
plot(t_est,  y_est,  "b", "LineWidth", 1.2);
plot(t_real, y_ref_i, "k--", "LineWidth", 1.4);
grid on;
ylabel("y [m]");
title("Posición Y");
legend("Real","Estimado","Referencia");

subplot(3,1,3);
plot(t_real, z_real, "g", "LineWidth", 1.3); hold on;
plot(t_est,  z_est,  "b", "LineWidth", 1.2);
plot(t_real, z_ref_i, "k--", "LineWidth", 1.4);
grid on;
xlabel("t [s]");
ylabel("z [m]");
title("Posición Z");
legend("Real","Estimado","Referencia");

sgtitle("Seguimiento de posición");

%% ============================================================
% 4. ERROR DE POSICIÓN INERCIAL
% ============================================================

ex = x_real - x_ref_i;
ey = y_real - y_ref_i;
ez = z_real - z_ref_i;

e_norm = sqrt(ex.^2 + ey.^2 + ez.^2);

figure;

subplot(2,1,1);
plot(t_real, ex, "LineWidth", 1.3); hold on;
plot(t_real, ey, "LineWidth", 1.3);
plot(t_real, ez, "LineWidth", 1.3);
grid on;
ylabel("Error [m]");
title("Error cartesiano de posición");
legend("e_x","e_y","e_z");

subplot(2,1,2);
plot(t_real, e_norm, "LineWidth", 1.5);
grid on;
xlabel("t [s]");
ylabel("||e_r|| [m]");
title("Norma del error de posición");

%% ============================================================
% 5. CONTROL APLICADO EN INERCIAL
% ============================================================

figure;

plot(t_u, u_MPC(:,1), "LineWidth", 1.3); hold on;
plot(t_u, u_MPC(:,2), "LineWidth", 1.3);
plot(t_u, u_MPC(:,3), "LineWidth", 1.3);
grid on;

xlabel("t [s]");
ylabel("u [m/s^2]");
title("Control aplicado por el MPC");
legend("u_x","u_y","u_z");

if exist("u_max","var")
    yline(u_max,  "r--", "u_{max}");
    yline(-u_max, "r--", "-u_{max}");
end

%% ============================================================
% 6. DISTANCIA AL DEBRIS
% ============================================================

if has_debris_traj || exist("rk_debris","var")

    if has_debris_traj
        dist_debris = sqrt((x_real - x_debris_i).^2 + ...
                           (y_real - y_debris_i).^2 + ...
                           (z_real - z_debris_i).^2);
    else
        dist_debris = sqrt((x_real - rk_debris_plot(1)).^2 + ...
                           (y_real - rk_debris_plot(2)).^2 + ...
                           (z_real - rk_debris_plot(3)).^2);
    end

    [dist_min, idx_min] = min(dist_debris);
    t_min = t_real(idx_min);

    figure;
    plot(t_real, dist_debris, "LineWidth", 1.5); hold on;

    if has_dsafe_log
        dsafe_first_i = interp1(t_dsafe, dsafe_first, t_real, "previous", "extrap");
        dsafe_horizon_i = interp1(t_dsafe, dsafe_horizon_max, t_real, "previous", "extrap");

        plot(t_real, dsafe_first_i, "m--", "LineWidth", 1.1);
        plot(t_real, dsafe_horizon_i, "c-.", "LineWidth", 1.1);

        if exist("dsafe0","var")
            yline(dsafe0, "r--", "d_{safe,0}");
            legend("Distancia al debris", "d_{safe} primer paso", "d_{safe} max horizonte", "d_{safe,0}");
        else
            legend("Distancia al debris", "d_{safe} primer paso", "d_{safe} max horizonte");
        end
    elseif exist("dsafe0","var")
        yline(dsafe0, "r--", "d_{safe}");
        legend("Distancia al debris", "Distancia segura");
    else
        legend("Distancia al debris");
    end

    grid on;
    xlabel("t [s]");
    ylabel("Distancia [m]");
    title(sprintf("Distancia al debris | mínima = %.3f m en t = %.2f s", dist_min, t_min));

end

if has_dsafe_log
    figure;
    plot(t_dsafe, dsafe_first, "m--", "LineWidth", 1.3); hold on;
    plot(t_dsafe, dsafe_horizon_max, "c-.", "LineWidth", 1.3);

    if exist("dsafe0", "var")
        yline(dsafe0, "r--", "d_{safe,0}");
        legend("d_{safe} primer paso", "d_{safe} max horizonte", "d_{safe,0}");
    else
        legend("d_{safe} primer paso", "d_{safe} max horizonte");
    end

    grid on;
    xlabel("t [s]");
    ylabel("Distancia [m]");
    title("Radio de seguridad dinamico usado por el MPC");
end

%% ============================================================
% 7. PROYECCIONES 2D: XY, XZ, YZ
% ============================================================

figure;

theta_c = linspace(0, 2*pi, 300);
circle_x = [];
circle_y = [];

if exist("dsafe0","var")
    circle_x = dsafe0*cos(theta_c);
    circle_y = dsafe0*sin(theta_c);
end

subplot(1,3,1);
hold on; grid on; axis equal;
plot(x_ref,  y_ref,  "k--", "LineWidth", 1.4);
plot(x_real, y_real, "g",   "LineWidth", 1.2);
plot(x_est,  y_est,  "b",   "LineWidth", 1.1);
if has_debris_traj
    plot(x_debris, y_debris, "r-", "LineWidth", 1.1);
    plot(rk_debris_plot(1), rk_debris_plot(2), "ro", "MarkerFaceColor", "r");
elseif exist("rk_debris","var")
    plot(rk_debris_plot(1), rk_debris_plot(2), "ro", "MarkerFaceColor", "r");
    if exist("dsafe0","var") && dsafe0 > 0
        plot(rk_debris_plot(1)+circle_x, rk_debris_plot(2)+circle_y, "r--", "LineWidth", 1.2);
    end
end
xlabel("x [m]");
ylabel("y [m]");
title("Plano XY");
if has_debris_traj
    legend("Referencia","Real","Estimado","Trayectoria debris","Debris en encuentro");
else
    legend("Referencia","Real","Estimado","Debris","Zona segura");
end

subplot(1,3,2);
hold on; grid on; axis equal;
plot(x_ref,  z_ref,  "k--", "LineWidth", 1.4);
plot(x_real, z_real, "g",   "LineWidth", 1.2);
plot(x_est,  z_est,  "b",   "LineWidth", 1.1);
if has_debris_traj
    plot(x_debris, z_debris, "r-", "LineWidth", 1.1);
    plot(rk_debris_plot(1), rk_debris_plot(3), "ro", "MarkerFaceColor", "r");
elseif exist("rk_debris","var")
    plot(rk_debris_plot(1), rk_debris_plot(3), "ro", "MarkerFaceColor", "r");
    if exist("dsafe0","var") && dsafe0 > 0
        plot(rk_debris_plot(1)+circle_x, rk_debris_plot(3)+circle_y, "r--", "LineWidth", 1.2);
    end
end
xlabel("x [m]");
ylabel("z [m]");
title("Plano XZ");

subplot(1,3,3);
hold on; grid on; axis equal;
plot(y_ref,  z_ref,  "k--", "LineWidth", 1.4);
plot(y_real, z_real, "g",   "LineWidth", 1.2);
plot(y_est,  z_est,  "b",   "LineWidth", 1.1);
if has_debris_traj
    plot(y_debris, z_debris, "r-", "LineWidth", 1.1);
    plot(rk_debris_plot(2), rk_debris_plot(3), "ro", "MarkerFaceColor", "r");
elseif exist("rk_debris","var")
    plot(rk_debris_plot(2), rk_debris_plot(3), "ro", "MarkerFaceColor", "r");
    if exist("dsafe0","var") && dsafe0 > 0
        plot(rk_debris_plot(2)+circle_x, rk_debris_plot(3)+circle_y, "r--", "LineWidth", 1.2);
    end
end
xlabel("y [m]");
ylabel("z [m]");
title("Plano YZ");

sgtitle("Proyecciones cartesianas");

%% ============================================================
% 8. ERROR Y CONTROL EN FRAME LVLH
% ============================================================

N = length(t_real);
x_rel_lvlh = zeros(N,3);

for k = 1:N

    r_abs_k = [x_real(k); y_real(k); z_real(k)];
    r_ref_k = [x_ref_i(k); y_ref_i(k); z_ref_i(k)];
    v_ref_k = [vx_ref_i(k); vy_ref_i(k); vz_ref_i(k)];

    [T_abs_to_ref, ~] = referenceFrameTransform(r_ref_k, v_ref_k);

    dr_abs_k = r_abs_k - r_ref_k;
    dr_lvlh_k = T_abs_to_ref * dr_abs_k;

    x_rel_lvlh(k,:) = dr_lvlh_k.';
end

u_MPC_lvlh = zeros(size(u_MPC));

for k = 1:length(t_u)

    r_ref_k = [ ...
        interp1(t_real, x_ref_i, t_u(k), "pchip", "extrap");
        interp1(t_real, y_ref_i, t_u(k), "pchip", "extrap");
        interp1(t_real, z_ref_i, t_u(k), "pchip", "extrap")];

    v_ref_k = [ ...
        interp1(t_real, vx_ref_i, t_u(k), "pchip", "extrap");
        interp1(t_real, vy_ref_i, t_u(k), "pchip", "extrap");
        interp1(t_real, vz_ref_i, t_u(k), "pchip", "extrap")];

    [T_abs_to_ref, ~] = referenceFrameTransform(r_ref_k, v_ref_k);

    u_abs_k = u_MPC(k,:).';
    u_MPC_lvlh(k,:) = (T_abs_to_ref * u_abs_k).';
end

figure;

subplot(3,1,1);
plot(t_real, x_rel_lvlh(:,1), "LineWidth", 1.3); hold on;
plot(t_u, u_MPC_lvlh(:,1)*1e4, "LineWidth", 1.1);
grid on;
ylabel("Radial");
legend("error [m]", "u radial x10^4");

subplot(3,1,2);
plot(t_real, x_rel_lvlh(:,2), "LineWidth", 1.3); hold on;
plot(t_u, u_MPC_lvlh(:,2)*1e4, "LineWidth", 1.1);
grid on;
ylabel("Tangencial");
legend("error [m]", "u tangencial x10^4");

subplot(3,1,3);
plot(t_real, x_rel_lvlh(:,3), "LineWidth", 1.3); hold on;
plot(t_u, u_MPC_lvlh(:,3)*1e4, "LineWidth", 1.1);
grid on;
xlabel("t [s]");
ylabel("Normal");
legend("error [m]", "u normal x10^4");

sgtitle("Error relativo y control en frame LVLH");

%% ============================================================
% 9. RESUMEN
% ============================================================

fprintf("\n===== RESUMEN POSTPROCESO MPC =====\n");

if exist("dist_debris","var")
    fprintf("Distancia mínima al debris: %.3f m\n", dist_min);
    if exist("dsafe0","var")
        fprintf("Distancia segura dsafe0:    %.3f m\n", dsafe0);
    end
    if has_dsafe_log
        fprintf("d_safe primer paso:        [%.3f, %.3f] m\n", min(dsafe_first), max(dsafe_first));
        fprintf("d_safe max horizonte:      [%.3f, %.3f] m\n", min(dsafe_horizon_max), max(dsafe_horizon_max));
    end
    fprintf("Instante distancia mínima:  %.3f s\n", t_min);
end

fprintf("Máximo |u_x|: %.6f m/s^2\n", max(abs(u_MPC(:,1))));
fprintf("Máximo |u_y|: %.6f m/s^2\n", max(abs(u_MPC(:,2))));
fprintf("Máximo |u_z|: %.6f m/s^2\n", max(abs(u_MPC(:,3))));

if exist("u_max","var")
    fprintf("Límite u_max: %.6f m/s^2\n", u_max);
end

fprintf("Error final posición: %.6f m\n", e_norm(end));
fprintf("Máximo error posición: %.6f m\n", max(e_norm));
fprintf("===================================\n\n");

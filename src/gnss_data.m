%% =========================================================
%  .DAT → ECEF + VELOCIDAD + MATRIZ DE COVARIANZA 6×6
%  para Simulink (timeseries)
% ==========================================================
%clear; clc;

%% -------------------------------------------------------
%  1. LEER ARCHIVO (.dat)
% -------------------------------------------------------
filename = inoas_data_file('cov_perturb_POS_s6a_Y24D011_fixed.dat');
data = readmatrix(filename, 'CommentStyle', '#');

%% -------------------------------------------------------
%  2. FILTRAR DATOS VÁLIDOS (SOL == 1)
% -------------------------------------------------------
data = data(data(:,7) == 1, :);

%% -------------------------------------------------------
%  3. EXTRAER COLUMNAS
%     Col:  1=SOD  2=LON  3=LAT  4=ALT  5=CLKEST  6=GGTO
%           7=SOL  8=NSVVIS  9=NSV
%           10=HPE 11=VPE 12=EPE 13=NPE 14=UPE
%           15=HDOP 16=VDOP 17=PDOP
% -------------------------------------------------------
t   = data(:,1);    % tiempo (SOD)
lon = data(:,2);    % longitud (grados)
lat = data(:,3);    % latitud  (grados)
alt = data(:,4);    % altitud  (metros)
nsv = data(:,8);    % número de satélites
epe = data(:,12);   % error Este  (m)
npe = data(:,13);   % error Norte (m)
upe = data(:,14);   % error Up    (m)

%% -------------------------------------------------------
%  4. NORMALIZAR TIEMPO (empezar en 0)
% -------------------------------------------------------
t = t - t(1);

%% -------------------------------------------------------
%  5. LLA → ECEF (WGS84)
% -------------------------------------------------------
phi    = deg2rad(lat);
lambda = deg2rad(lon);
h      = alt;

a = 6378137.0;              % radio ecuatorial (m)
e = 8.1819190842622e-2;     % excentricidad

N = a ./ sqrt(1 - e^2 .* sin(phi).^2);

x = (N + h) .* cos(phi) .* cos(lambda);
y = (N + h) .* cos(phi) .* sin(lambda);
z = (N .* (1 - e^2) + h) .* sin(phi);

%% -------------------------------------------------------
%  6. VELOCIDAD (derivada numérica)
% -------------------------------------------------------
dt = mean(diff(t));     % intervalo de muestreo (s)

vx = gradient(x, t);
vy = gradient(y, t);
vz = gradient(z, t);

% Corregir primer punto (artefacto de gradient en el borde)
vx(1) = vx(2);
vy(1) = vy(2);
vz(1) = vz(2);

%% -------------------------------------------------------
%  7. ESTADÍSTICAS DE ERROR POR NSV
% -------------------------------------------------------
sat_groups = unique(nsv);
fprintf('\n--- Estadísticas de error por Nº satélites ---\n');
fprintf('%-6s  %-10s  %-10s  %-10s  %-6s\n', 'NSV', 'σ_N (m)', 'σ_E (m)', 'σ_U (m)', 'N_obs');
for k = 1:length(sat_groups)
    idx = nsv == sat_groups(k);
    fprintf('%-6d  %-10.4f  %-10.4f  %-10.4f  %-6d\n', ...
        sat_groups(k), std(npe(idx)), std(epe(idx)), std(upe(idx)), sum(idx));
end

%% -------------------------------------------------------
%  8. PRECOMPUTAR Q_6×6 PARA CADA TIMESTAMP
% -------------------------------------------------------
n_samples = length(t);
Q_all  = zeros(6, 6, n_samples);
Q_flat = zeros(n_samples, 36);

for i = 1:n_samples

    % --- 8a. Sigma por grupo NSV de esta época ---
    idx_nsv = nsv == nsv(i);

    % Mínimo 2 muestras para calcular std, si no usar global
    if sum(idx_nsv) >= 2
        sigma_n = std(npe(idx_nsv));
        sigma_e = std(epe(idx_nsv));
        sigma_u = std(upe(idx_nsv));
    else
        sigma_n = std(npe);
        sigma_e = std(epe);
        sigma_u = std(upe);
    end

    % Evitar sigma = 0 (época con datos perfectos o sin variación)
    sigma_n = max(sigma_n, 1e-4);
    sigma_e = max(sigma_e, 1e-4);
    sigma_u = max(sigma_u, 1e-4);

    % --- 8b. Covarianza local NEU ---
    Q_neu = diag([sigma_n^2, sigma_e^2, sigma_u^2]);

    % --- 8c. Matriz de rotación NEU → ECEF ---
    la = phi(i);
    lo = lambda(i);

    R = [ -sin(la)*cos(lo),  -sin(lo),  cos(la)*cos(lo) ;
          -sin(la)*sin(lo),   cos(lo),  cos(la)*sin(lo) ;
           cos(la),           0,        sin(la)         ];

    % --- 8d. Covarianza posición en ECEF ---
    Q_pos = R * Q_neu * R';

    % --- 8e. Covarianza velocidad (propagación desde posición) ---
    sigma_vn = sigma_n / dt * sqrt(2);
    sigma_ve = sigma_e / dt * sqrt(2);
    sigma_vu = sigma_u / dt * sqrt(2);

    Q_vel_neu = diag([sigma_vn^2, sigma_ve^2, sigma_vu^2]);
    Q_vel = R * Q_vel_neu * R';

    % --- 8f. Ensamblar Q_6×6 ---
    Q_6x6 = zeros(6,6);
    Q_6x6(1:3, 1:3) = Q_pos;
    Q_6x6(4:6, 4:6) = Q_vel;

    Q_all(:,:,i) = Q_6x6;

    % --- 8g. Aplanar fila por fila (column-major, reshape por defecto MATLAB) ---
    Q_flat(i,:) = reshape(Q_6x6, 1, 36);
end

%% -------------------------------------------------------
%  9. VERIFICACIÓN: SDP de la primera y última Q
% -------------------------------------------------------
fprintf('\n--- Verificación Q_6×6 ---\n');
for idx_check = [1, n_samples]
    eigs_q = eig(Q_all(:,:,idx_check));
    if all(eigs_q >= -1e-10)
        fprintf('Época %d → Semidefinida positiva ✓\n', idx_check);
    else
        warning('Época %d → NO semidefinida positiva ⚠', idx_check);
    end
end

%% -------------------------------------------------------
%  10. CREAR TIMESERIES PARA SIMULINK
% -------------------------------------------------------
% Posición ECEF
ts_pos = timeseries([x y z], t);
ts_pos.Name = 'ECEF_Position';
ts_pos.DataInfo.Units = 'm';

% Velocidad ECEF
ts_vel = timeseries([vx vy vz], t);
ts_vel.Name = 'ECEF_Velocity';
ts_vel.DataInfo.Units = 'm/s';

% Covarianza aplanada (36 elementos por época)
ts_Q = timeseries(Q_flat, t);
ts_Q.Name = 'CovarianceMatrix_flat36';
ts_Q.DataInfo.Units = 'm^2';

disp('✔ Timeseries listos para Simulink:');
fprintf('  ts_pos  → [x, y, z]          (%d × 3)\n', n_samples);
fprintf('  ts_vel  → [vx, vy, vz]        (%d × 3)\n', n_samples);
fprintf('  ts_Q    → Q aplanada 6×6      (%d × 36)\n', n_samples);

%% -------------------------------------------------------
%  11. VISUALIZACIÓN
% -------------------------------------------------------
figure('Name', 'ECEF Position & Velocity');
subplot(2,1,1)
    plot(t, x, t, y, t, z); grid on;
    legend('x','y','z'); title('Posición ECEF (m)'); xlabel('t (s)');
subplot(2,1,2)
    plot(t, vx, t, vy, t, vz); grid on;
    legend('vx','vy','vz'); title('Velocidad ECEF (m/s)'); xlabel('t (s)');

figure('Name', 'Diagonal Q_6×6 por época');
Q_diag_plot = zeros(n_samples, 6);
for i = 1:n_samples
    Q_diag_plot(i,:) = diag(Q_all(:,:,i))';
end
labels = {'σ²_x','σ²_y','σ²_z','σ²_{vx}','σ²_{vy}','σ²_{vz}'};
for k = 1:6
    subplot(2,3,k)
    plot(t, Q_diag_plot(:,k)); grid on;
    title(labels{k}); xlabel('t (s)');
end
sgtitle('Diagonal de Q_{6×6} por época');


% %% =========================================================
% %  .DAT → ECEF (x,y,z) + VELOCIDAD (vx,vy,vz)
% % ==========================================================
% 
% clear; clc;
% 
% %% 1. Leer archivo (.dat)
% filename = 'perturb_POS_s6a_Y24D011.dat';
% data = readmatrix(filename, 'CommentStyle', '#');
% 
% %% 2. Filtrar datos válidos (SOL == 1)
% data = data(data(:,7) == 1,:);
% 
% %% 3. Extraer columnas
% t   = data(:,1);   % tiempo (SOD)
% lon = data(:,2);   % grados
% lat = data(:,3);   % grados
% alt = data(:,4);   % metros
% 
% %% 4. Normalizar tiempo (empezar en 0)
% t = t - t(1);
% 
% %% 5. LLA → ECEF (WGS84)
% phi    = deg2rad(lat);
% lambda = deg2rad(lon);
% h      = alt;
% 
% a = 6378137.0;                % radio ecuatorial (m)
% e = 8.1819190842622e-2;       % excentricidad
% 
% N = a ./ sqrt(1 - e^2 .* sin(phi).^2);
% 
% x = (N + h) .* cos(phi) .* cos(lambda);
% y = (N + h) .* cos(phi) .* sin(lambda);
% z = (N .* (1 - e^2) + h) .* sin(phi);
% 
% %% 6. Velocidad (derivada numérica)
% vx = gradient(x, t);
% vy = gradient(y, t);
% vz = gradient(z, t);
% 
% %% 7. Corregir primer punto (evitar artefacto inicial)
% vx(1) = vx(2);
% vy(1) = vy(2);
% vz(1) = vz(2);
% 
% %% 8. Crear timeseries para Simulink
% ts_pos = timeseries([x y z], t);
% ts_pos.Name = 'ECEF position';
% ts_pos.DataInfo.Units = 'm';
% 
% ts_vel = timeseries([vx vy vz], t);
% ts_vel.Name = 'ECEF velocity';
% ts_vel.DataInfo.Units = 'm/s';
% 
% %% 9. Check rápido
% disp('✔ Datos listos para Simulink');
% disp(['Samples: ', num2str(length(t))]);
% 
% %% 10. (Opcional) visualización
% figure;
% subplot(2,1,1)
% plot(t, x, t, y, t, z);
% legend('x','y','z'); grid on;
% title('Position ECEF');
% 
% subplot(2,1,2)
% plot(t, vx, t, vy, t, vz);
% legend('vx','vy','vz'); grid on;
% title('Velocity ECEF');


% %% =========================================================
% %  PREPARACIÓN DE DATOS NAV DESDE .DAT → ECEF (x,y,z)
% % ==========================================================
% 
% clear; clc;
% 
% %% 1. Leer archivo .dat (ignorar cabecera con '#')
% filename = 'perturb_POS_s6a_Y24D011.dat';
% 
% data = readmatrix(filename, 'CommentStyle', '#');
% 
% %% 2. Filtrar datos válidos (SOL == 1)
% % Columna 7 según tu formato
% sol = data(:,7);
% data = data(sol == 1,:);
% 
% %% 3. Extraer columnas
% t   = data(:,1);   % tiempo (SOD) en segundos
% lon = data(:,2);   % grados
% lat = data(:,3);   % grados
% alt = data(:,4);   % metros
% 
% %% 4. Convertir a radianes
% phi     = deg2rad(lat);   % latitud
% lambda  = deg2rad(lon);   % longitud
% h       = alt;
% 
% %% 5. Constantes WGS84
% a = 6378137.0;                 % radio ecuatorial (m)
% e = 8.1819190842622e-2;        % excentricidad
% 
% %% 6. LLA → ECEF
% N = a ./ sqrt(1 - e^2 .* sin(phi).^2);
% 
% x = (N + h) .* cos(phi) .* cos(lambda);
% y = (N + h) .* cos(phi) .* sin(lambda);
% z = (N .* (1 - e^2) + h) .* sin(phi);
% 
% %% 7. Crear timeseries para Simulink
% ts_ecef = timeseries([x y z], t);
% ts_ecef.Name = 'ECEF position';
% ts_ecef.DataInfo.Units = 'm';

% clear; clc;
% 
% %% Leer .dat
% data = readmatrix('perturb_POS_s6a_Y24D011.dat', 'CommentStyle', '#');
% 
% %% Filtrar datos válidos
% data = data(data(:,7) == 1,:);
% 
% %% Extraer
% t   = data(:,1);
% lon = data(:,2);
% lat = data(:,3);
% alt = data(:,4);
% 
% %% Tiempo desde 0
% t = t - t(1);
% 
% %% LLA → ECEF
% phi = deg2rad(lat);
% lambda = deg2rad(lon);
% h = alt;
% 
% a = 6378137.0;
% e = 8.1819190842622e-2;
% 
% N = a ./ sqrt(1 - e^2 .* sin(phi).^2);
% 
% x = (N + h) .* cos(phi) .* cos(lambda);
% y = (N + h) .* cos(phi) .* sin(lambda);
% z = (N .* (1 - e^2) + h) .* sin(phi);
% 
% %% Timeseries
% ts_ecef = timeseries([x y z], t);

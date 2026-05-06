function x_next = myStateTransitionFcn(x_current, u)
    % =========================================================================
    % STATE TRANSITION FUNCTION FOR UNSCENTED KALMAN FILTER (UKF)
    % =========================================================================
    % LAYOUT GENERAL Y ARQUITECTURA:
    % Esta función actúa como el modelo predictivo de la planta dentro del UKF.
    % Se divide en dos componentes principales para garantizar precisión y
    % compatibilidad con la generación de código C en Simulink:
    %
    % 1. El Integrador Numérico (Función Principal): 
    %    Dado que el UKF opera en tiempo discreto y ode45 no es compatible 
    %    con la generación de código embebido, se implementa un integrador 
    %    Runge-Kutta de 4º orden (RK4) de paso fijo. Este método evalúa la 
    %    física del sistema en 4 puntos del intervalo 'dt' para minimizar 
    %    el error de truncamiento al propagar la órbita.
    %
    % 2. El Motor Físico (Subfunción 'get_state_derivative'):
    %    Calcula la derivada instantánea del estado (velocidad y aceleración). 
    %    Modela el campo gravitatorio terrestre sumando la gravedad central 
    %    de Kepler y las perturbaciones debidas al achatamiento de la Tierra. 
    %    Utiliza la formulación analítica general de los armónicos zonales 
    %    (J2, J4, J6) basada en polinomios de Legendre (Eq. 5.2), permitiendo 
    %    escalar la fidelidad del modelo dinámico con solo modificar un vector.
    % =========================================================================
    
    % --- 0. SEGURIDAD DE DIMENSIONES SIMULINK ---
    % Forzamos a que la señal de control sea un vector columna (3x1)
    u = u(:);
    
    % --- 1. CONFIGURACIÓN DEL PASO DE TIEMPO ---
    dt = 1; % [s] IMPORTANTE: Debe coincidir con tu Sample Time (Ts)
    
    % --- 2. INTEGRADOR RUNGE-KUTTA 4 (RK4) ---
    % Evaluamos la derivada en 4 puntos a lo largo del paso de integración dt
    % Le pasamos también la señal de control 'u' (constante durante el paso dt)
    k1 = get_state_derivative(x_current, u);
    k2 = get_state_derivative(x_current + 0.5 * dt * k1, u);
    k3 = get_state_derivative(x_current + 0.5 * dt * k2, u);
    k4 = get_state_derivative(x_current + dt * k3, u);
    
    % Calculamos el estado final propagado ponderando las evaluaciones
    x_next = x_current + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

% =========================================================================
% SUBFUNCIÓN: Derivada usando Polinomios de Legendre (Eq. 5.2)
% =========================================================================
function x_dot = get_state_derivative(x, u)
    % 1. Parámetros de la Tierra (WGS84)
    mu_earth = 3.986004418e14; % Parámetro gravitacional [m^3/s^2]
    R_earth = 6378137.0;       % Radio ecuatorial [m]
    
    % Vector de coeficientes zonales [J2, J4, J6]. 
    % Mantén J4 y J6 a cero si de momento solo buscas iterar con J2.
    J = [1.08262668e-3, 0, 0]; 
    n_vals = [2, 4, 6];        
    
    % 2. Desempaquetar el vector de estado
    r_vec = x(1:3); % Vector de posición [m] (Aseguramos columna)
    r_vec = r_vec(:);
    v_vec = x(4:6); % Vector de velocidad [m/s] (Aseguramos columna)
    v_vec = v_vec(:);
    
    z = r_vec(3);
    r = norm(r_vec);
    
    % u_z previene colisión de nombres con la variable de control 'u'
    u_z = z / r; 
    z_hat = [0; 0; 1]; % Vector unitario 'k'
    
    % 3. Primer término: Aceleración central (Kepler)
    a_central = -(mu_earth / r^3) * r_vec;
    
    % 4. Segundo término: Sumatorio de perturbaciones armónicas zonales
    a_zonales = zeros(3,1); % Inicializamos el vector de perturbación
    
    for i = 1:length(n_vals)
        n = n_vals(i);
        Jn = J(i);
        
        if Jn == 0
            continue; % Optimización: saltamos el cálculo si el coeficiente es cero
        end
        
        % Evaluar el polinomio de Legendre Pn(u_z) y su derivada dPn/du_z
        if n == 2
            Pn  = 0.5 * (3*u_z^2 - 1);
            dPn = 3*u_z;
        elseif n == 4
            Pn  = (1/8) * (35*u_z^4 - 30*u_z^2 + 3);
            dPn = 0.5 * (140*u_z^3 - 60*u_z);
        elseif n == 6
            Pn  = (1/16) * (231*u_z^6 - 315*u_z^4 + 105*u_z^2 - 5);
            dPn = (1/16) * (1386*u_z^5 - 1260*u_z^3 + 210*u_z);
        else
            Pn = 0; dPn = 0;
        end
        
        % TRADUCCIÓN LITERAL DE LA ECUACIÓN 5.2
        termino_exterior = (mu_earth / r^2) * (R_earth / r)^n * Jn;
        corchete = (n + 1) * Pn * (r_vec / r) - dPn * ((z / r^2) * r_vec - z_hat);
        
        % Acumulamos la contribución de este armónico a la aceleración total
        a_zonales = a_zonales + termino_exterior * corchete;
    end
    
    % 5. Aceleración neta experimentada por el satélite
    % Sumamos la gravedad natural MÁS tu vector de control (u)
    a_total = a_central + a_zonales + u;
    
    % 6. Ensamblar y devolver la derivada del estado [v; a]
    x_dot = [v_vec; a_total];
end
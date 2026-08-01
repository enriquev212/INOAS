function y = myMeasurementFcn(x)
    % x es el vector de estado predicho: [pos_x; pos_y; pos_z; vel_x; vel_y; vel_z]
    
    % 1. Extraemos las posiciones
    pos_x = x(1);
    pos_y = x(2);
    pos_z = x(3);
    
    % 2. Calculamos la altitud
    R_earth = 6378137.0; % Radio de la Tierra en metros
    r_norm = sqrt(pos_x^2 + pos_y^2 + pos_z^2);
    altitud = r_norm - R_earth;
    
    % 3. Ensamblamos el vector de medida de salida (4x1)
    y = [pos_x; pos_y; pos_z; altitud];
end
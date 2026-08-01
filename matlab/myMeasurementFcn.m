function y = myMeasurementFcn(x)
    % Predicted state vector: [pos_x; pos_y; pos_z; vel_x; vel_y; vel_z].

    % 1. Extract position components.
    pos_x = x(1);
    pos_y = x(2);
    pos_z = x(3);

    % 2. Compute altitude above the Earth reference radius.
    R_earth = 6378137.0; % [m]
    r_norm = sqrt(pos_x^2 + pos_y^2 + pos_z^2);
    altitude = r_norm - R_earth;

    % 3. Assemble the output measurement vector [4x1].
    y = [pos_x; pos_y; pos_z; altitude];
end

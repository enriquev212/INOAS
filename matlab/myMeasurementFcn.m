function y = myMeasurementFcn(x)
    % Predicted state vector: [pos_x; pos_y; pos_z; vel_x; vel_y; vel_z].
    % Auxiliary measurement model: position only.

    % 1. Extract position components.
    pos_x = x(1);
    pos_y = x(2);
    pos_z = x(3);

    % 2. Assemble the output measurement vector [3x1].
    % The altitude channel was removed: it was h = |r| - R_earth, a function of
    % the same position, so it carried no independent information and made the
    % diagonal R inconsistent.
    y = [pos_x; pos_y; pos_z];
end

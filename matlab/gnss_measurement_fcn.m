function y = gnss_measurement_fcn(x)
    %#codegen

    % Preallocate the measurement vector as a 6x1 column.
    y = zeros(6,1);

    % The GNSS measurement exposes the full position-velocity state.
    y(1) = x(1);
    y(2) = x(2);
    y(3) = x(3);
    y(4) = x(4);
    y(5) = x(5);
    y(6) = x(6);
end

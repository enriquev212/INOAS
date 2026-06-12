function y = medida_gnss_fcn(x)
    %#codegen
    % 1. Pre-asignamos la memoria obligando a que sea un vector columna 6x1
    y = zeros(6,1);
    
    % 2. Asignamos los valores uno a uno
    y(1) = x(1);
    y(2) = x(2);
    y(3) = x(3);
    y(4) = x(4);
    y(5) = x(5);
    y(6) = x(6);
end
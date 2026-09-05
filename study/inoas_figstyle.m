function C = inoas_figstyle()
%INOAS_FIGSTYLE Estilo comun de las figuras del paper.
%
%   Devuelve la paleta y fija los valores por defecto. Se llama una vez al
%   principio de cada script de figuras; el acabado por eje lo aplica
%   inoas_axstyle despues de dibujar.
%
%   Criterios: caja abierta (solo ejes izquierdo e inferior), marcas hacia
%   fuera, rejilla horizontal muy tenue y por detras, y trazo mas grueso que el
%   de MATLAB por defecto, que a 3.5 in de ancho queda anemico al imprimir.

    set(groot, 'defaultAxesFontName', 'Times New Roman');
    set(groot, 'defaultTextFontName', 'Times New Roman');
    set(groot, 'defaultAxesFontSize', 8);
    set(groot, 'defaultAxesLabelFontSizeMultiplier', 1.0);
    set(groot, 'defaultLineLineWidth', 1.3);
    set(groot, 'defaultAxesLineWidth', 0.6);
    set(groot, 'defaultAxesTickDir', 'out');
    set(groot, 'defaultAxesTickDirMode', 'manual');
    set(groot, 'defaultAxesBox', 'off');
    set(groot, 'defaultAxesXColor', [0.15 0.15 0.15]);
    set(groot, 'defaultAxesYColor', [0.15 0.15 0.15]);
    set(groot, 'defaultFigureColor', 'w');

    % Paleta segura para daltonismo y legible en escala de grises
    C.blue = [0.00 0.42 0.68];
    C.red  = [0.80 0.16 0.13];
    C.grn  = [0.00 0.50 0.36];
    C.orn  = [0.88 0.52 0.06];
    C.pur  = [0.45 0.24 0.55];
    C.gry  = [0.42 0.42 0.42];
    C.lgry = [0.72 0.72 0.72];
    C.order = [C.blue; C.grn; C.orn; C.red; C.pur];
end

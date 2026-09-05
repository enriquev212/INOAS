function inoas_axstyle(ax, opts)
%INOAS_AXSTYLE Acabado del eje, aplicado despues de dibujar.
%
%   MATLAB por defecto encierra el eje en una caja, mete las marcas hacia dentro
%   en los cuatro lados y pone una rejilla que compite con los datos. En una
%   figura de 3.5 in eso satura. Aqui: caja abierta, marcas hacia fuera y solo
%   en los ejes visibles, rejilla horizontal punteada y por detras de los datos.
%
%   opts.grid : 'y' (por defecto), 'xy' o 'none'
%   opts.legendLoc : posicion de la leyenda, si existe

    arguments
        ax = gca
        opts.grid (1,:) char = 'y'
        opts.legendLoc (1,:) char = ''
    end

    set(ax, 'Box', 'off', 'TickDir', 'out', 'TickLength', [0.014 0.014], ...
            'LineWidth', 0.6, 'FontName', 'Times New Roman', 'FontSize', 8, ...
            'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
            'Layer', 'bottom');

    % En escala logaritmica MATLAB anade una rejilla menor densisima que en una
    % figura de columna simple se lee como una trama de fondo.
    set(ax, 'XMinorGrid', 'off', 'YMinorGrid', 'off', ...
            'XMinorTick', 'off', 'YMinorTick', 'off');

    ax.XGrid = 'off'; ax.YGrid = 'off';
    switch lower(opts.grid)
        case 'y',  ax.YGrid = 'on';
        case 'xy', ax.XGrid = 'on'; ax.YGrid = 'on';
    end
    ax.GridLineStyle = ':';
    ax.GridColor = [0.55 0.55 0.55];
    ax.GridAlpha = 0.35;

    ax.XTickLabelRotation = 0;
    ax.YTickLabelRotation = 0;

    lg = findobj(ax.Parent, 'Type', 'Legend');
    if ~isempty(lg)
        set(lg, 'Box', 'off', 'FontSize', 7, 'FontName', 'Times New Roman', ...
                'ItemTokenSize', [14 8]);
        if ~isempty(opts.legendLoc); set(lg, 'Location', opts.legendLoc); end
    end
end

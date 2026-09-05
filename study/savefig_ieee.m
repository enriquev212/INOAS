function savefig_ieee(fig, name, outDir, W, H)
%SAVEFIG_IEEE Guarda una figura en formato de columna simple IEEE.
%   PDF vectorial para LaTeX y PNG a 300 dpi para revisar.
    if nargin < 4; W = 3.5; end     % pulgadas, columna simple IEEE
    if nargin < 5; H = 2.6; end
    set(fig, 'Units','inches', 'Position',[1 1 W H], ...
             'PaperUnits','inches', 'PaperSize',[W H], ...
             'PaperPosition',[0 0 W H], 'Color','w');
    exportgraphics(fig, fullfile(outDir, [name '.pdf']), 'ContentType','vector');
    exportgraphics(fig, fullfile(outDir, [name '.png']), 'Resolution', 300);
    close(fig);
    fprintf('  %s.pdf / .png\n', name);
end

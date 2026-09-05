function rd = inoas_debris_pos(hist, idx)
%INOAS_DEBRIS_POS Posicion del debris en el paso idx (hist es 6xN o vector apilado).
    if isempty(hist); rd = [inf;inf;inf]; return; end
    if size(hist,2) == 1
        n = numel(hist)/6; idx = min(max(idx,1),n);
        rd = hist((idx-1)*6 + (1:3));
    else
        idx = min(max(idx,1),size(hist,2));
        rd = hist(1:3,idx);
    end
    rd = rd(:);
end

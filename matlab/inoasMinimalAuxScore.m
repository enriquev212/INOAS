function [score, raw_error] = inoasMinimalAuxScore(z_aux, x_hat, R_aux)
%#codegen
% Post-fit, R-normalized auxiliary discrepancy, NOT a statistical NIS.
pos = x_hat(1:3);
predicted = [pos; norm(pos) - 6378137.0];
residual = z_aux(:) - predicted;
raw_error = zeros(6, 1);
raw_error(1:4) = residual;
if any(~isfinite(residual))
    score = Inf;
else
    score = residual.' * (R_aux \ residual);
end
end

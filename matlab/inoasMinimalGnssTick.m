function fresh = inoasMinimalGnssTick(t, cfg)
%#codegen
% Only for the present ideal, periodic generator, executed once per Ts.
elapsed = t - cfg.fixEpoch;
tick = round(elapsed / cfg.fixInterval);
fresh = elapsed >= 0 && ...
    abs(elapsed - tick * cfg.fixInterval) <= 1e-8;
end

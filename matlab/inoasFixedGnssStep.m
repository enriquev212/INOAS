function [lambda, receiver_on, mode, quality_ok] = ...
    inoasFixedGnssStep(aux_score, n_sat, PDOP, HPE, VPE, gnss_sol, t, cfg)
%#codegen
% Fixed 96 s ON / 300 s OFF calendar for the baseline policy.
persistent state enteredAt
OFF = uint8(0); ACQUIRING = uint8(1); TRACKING = uint8(2);
if isempty(state)
    state = OFF;
    enteredAt = t;
end
quality_ok = all(isfinite([n_sat, PDOP, gnss_sol])) && ...
    n_sat >= cfg.nsvMin && PDOP > 0 && PDOP <= cfg.pdopMax && gnss_sol >= 0.5;
receiver_on = t >= 0 && mod(t, cfg.fixedOnDuration + cfg.offTime) < cfg.fixedOnDuration;
fresh = inoasMinimalGnssTick(t, cfg);
if ~receiver_on
    state = OFF;
    enteredAt = t;
elseif state == OFF
    state = ACQUIRING;
    enteredAt = t;
elseif state == TRACKING && ~quality_ok
    state = ACQUIRING;
    enteredAt = t;
elseif state == ACQUIRING && t-enteredAt >= cfg.acquisitionTime && fresh && quality_ok
    state = TRACKING;
    enteredAt = t;
end
mode = state;
lambda = state == TRACKING;
end

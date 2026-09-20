function [lambda, receiver_on, mode, quality_ok] = ...
    inoasMinimalGnssStep(aux_score, n_sat, PDOP, HPE, VPE, gnss_sol, t, cfg)
%#codegen
% OFF=0, ACQUIRING=1, TRACKING=2. Call exactly once per estimator step.
% HPE/VPE are retained for interface compatibility, not used for acceptance:
% dataset reference errors are unavailable to an autonomous receiver.
persistent state enteredAt
OFF = uint8(0);
ACQUIRING = uint8(1);
TRACKING = uint8(2);
if isempty(state)
    state = ACQUIRING;
    enteredAt = t;
end

quality_ok = all(isfinite([n_sat, PDOP, gnss_sol])) && ...
    n_sat >= cfg.nsvMin && PDOP > 0 && PDOP <= cfg.pdopMax && ...
    gnss_sol >= 0.5;
aux_alarm = ~isfinite(aux_score) || aux_score >= cfg.auxThreshold;
fresh = inoasMinimalGnssTick(t, cfg);
elapsed = t - enteredAt;

switch state
    case OFF
        % No GNSS-quality observation is required to request power-on.
        if elapsed >= cfg.offTime || aux_alarm
            state = ACQUIRING;
            enteredAt = t;
        end
    case ACQUIRING
        if elapsed >= cfg.acquisitionTime && fresh && quality_ok
            state = TRACKING;
            enteredAt = t;
        end
    case TRACKING
        if ~quality_ok
            state = ACQUIRING;
            enteredAt = t;
        elseif elapsed >= cfg.minTrackingTime && ~aux_alarm
            % A persistent alarm extends ON time, instead of OFF/ON chatter.
            state = OFF;
            enteredAt = t;
        end
    otherwise
        state = ACQUIRING;
        enteredAt = t;
end
mode = state;
lambda = state == TRACKING;
receiver_on = state ~= OFF;
end

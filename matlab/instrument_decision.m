function [lambda, nav_status] = instrument_decision(score, n_sat, PDOP, HPE, ...
    VPE, gnss_sol, onDuration, offDuration, threshold, dt)
%INSTRUMENT_DECISION GNSS supervisor with an explicit forecast snapshot.
% nav_status = [commanded lambda; elapsed time in mode; current GNSS health].
% Both outputs undergo the same one-sample delay in Simulink.
%#codegen
persistent active elapsed;
if isempty(active)
    active = true;
    elapsed = 0;
end
healthy = n_sat >= 4 && gnss_sol >= 0.5 && PDOP <= 6 && HPE <= 5 && VPE <= 5;
[active, elapsed] = navigationDutyCycleStep(active, elapsed, healthy, score, ...
    onDuration, offDuration, threshold, dt);
lambda = active;
nav_status = [double(active); elapsed; double(healthy)];
end

%OPEN_INOAS_DEBRIS_DEMO Open INOAS in reduced-horizon debris-demo mode.
%
% The configured debris encounter occurs at t = 800 s. This mode keeps the
% reduced MPC horizon used by open_inoas_fast.m, but runs long enough to inspect
% the avoidance maneuver around the encounter.

demoMpcConfig = struct();
demoMpcConfig.Np = 25;
simulationStopTime = 950;

setpref("inoas", "mpcTuneConfig", demoMpcConfig);

open_inoas_model;

set_param("inoas_model", "StopTime", "950");

fprintf("\nDebris-demo mode enabled: Np = %d, StopTime = 950 s, debris encounter at t = 800 s\n", ...
    25);

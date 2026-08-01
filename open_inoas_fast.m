%OPEN_INOAS_FAST Open INOAS with a reduced MPC horizon for quick validation.
%
% This mode is intended for smoke tests and setup checks. Use
% open_inoas_model.m for final scenario runs.

fastMpcConfig = struct();
fastMpcConfig.Np = 25;

setpref("inoas", "mpcTuneConfig", fastMpcConfig);

open_inoas_model;

set_param("inoas_model", "StopTime", "120");

fprintf("\nFast validation mode enabled: Np = %d, StopTime = 120 s\n", ...
    25);

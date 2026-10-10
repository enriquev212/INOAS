%OPEN_INOAS_FAST Open a short run without changing the MPC horizon.
%
% This mode is intended for smoke tests and setup checks. Use
% open_inoas_model.m for final scenario runs.

simulationStopTime = 120;

run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'open_inoas_model.m'));

set_param("inoas_model", "StopTime", "120");

fprintf("\nShort setup check: Np = 60, StopTime = 120 s\n");

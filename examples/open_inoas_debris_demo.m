%OPEN_INOAS_DEBRIS_DEMO Open the paper model through the encounter.
%
% Retains the 60-step / 720 s horizon and the design encounter epoch at 1500 s.

simulationStopTime = 1800;

run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'open_inoas_model.m'));

set_param("inoas_model", "StopTime", "1800");

fprintf("\nDebris demo: Np = 60, StopTime = 1800 s, encounter design epoch = 1500 s\n");

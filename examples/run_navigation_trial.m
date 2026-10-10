%RUN_NAVIGATION_TRIAL Shorter AUX3 paper-model run, including the first rejection.
addpath(fileparts(fileparts(mfilename('fullpath'))));
outputDir = run_inoas_case('reactive', 'adaptive', 'StopTime', 1000);
fprintf('Navigation trial exported to %s\n', outputDir);

%% RUN_DUTY_CYCLE_DIAGNOSTICS
% Applies the real GNSS/Kalman duty-cycle logic and generates diagnostic
% plots to separate tracking error from navigation/estimation error.

clearvars;
close all;
bdclose all;

thisFile = mfilename("fullpath");
thisDir = fileparts(thisFile);
cd(thisDir);

modelName = "MPCcontrolledSpacecraft_plant_gnss_kalman_decision";
stopTime = 600; % [s]

fprintf("\n==== GNSS/Kalman duty-cycle diagnostic ====\n");
fprintf("Model    : %s\n", modelName);
fprintf("StopTime : %.0f s\n\n", stopTime);

% Keep the existing selector wiring and Kalman correction architecture.
apply_kalman_architecture_fix(modelName);

% Push the current instrument_decision.m code into the embedded MATLAB
% Function block. Editing the .m file alone is not enough for Simulink.
apply_instrument_duty_cycle_fix(modelName);

% Run the simulation and create figures.
report = diagnose_kalman_gnss_selection(stopTime);

fprintf("\n==== Tracking error: signal - reference ====\n");
disp(report.summary);

fprintf("\n==== Navigation error: signal - plant ====\n");
disp(report.navigationError);

fprintf("\nFigures written to:\n");
fprintf("  Combined   : %s\n", report.figPath);
fprintf("  Tracking   : %s\n", report.trackingFigPath);
fprintf("  Navigation : %s\n", report.navigationFigPath);
fprintf("  Lambda     : %s\n", report.lambdaFigPath);
fprintf("  Components : %s\n", report.componentsFigPath);
fprintf("  MAT data   : %s\n", report.matPath);

open(report.navigationFigPath);
open(report.lambdaFigPath);

function outputDir = run_navigation_smoke_test()
%RUN_NAVIGATION_SMOKE_TEST Explicit 120 s Simulink integration check.
% Keep the 60-step paper horizon, and exercise acquisition at 36 s.
outputDir = run_inoas_case('reactive', 'adaptive', 'StopTime', 120, 'Seed', 7);
navigation = readtable(fullfile(outputDir, 'navigation.csv'));
assert(any(navigation.receiver_mode == 1));
assert(any(navigation.receiver_mode == 2));
assert(any(navigation.receiver_mode == 0));
firstFix = navigation.time_s(find(navigation.lambda > 0.5, 1));
assert(abs(firstFix-36) < 1e-8);
assert(all(isfinite(navigation.receiver_energy_Wh)));
fprintf('AUX3 acquisition, tracking, OFF transition and CSV export passed.\n');
end

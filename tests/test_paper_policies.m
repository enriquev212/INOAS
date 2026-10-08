function tests = test_paper_policies
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.oldPath = path;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'matlab'));
end

function teardownOnce(testCase)
path(testCase.TestData.oldPath);
clear inoasMinimalGnssStep inoasFixedGnssStep
end

function testDefaultPaperConfiguration(testCase)
cfg = inoasPaperConfig();
verifyEqual(testCase, cfg.ReceiverPolicy, 'reactive');
verifyEqual(testCase, cfg.RadiusMode, 'adaptive');
verifyEqual(testCase, cfg.StopTime, 6743);
verifyEqual(testCase, cfg.AcquisitionTime, 35);
end

function testReactiveNeverPowersOffDuringAcquisition(testCase)
clear inoasMinimalGnssStep inoasFixedGnssStep
cfg = inoasMinimalGnssConfig(1, 3);
for t = 0:1000
    [lambda, on, state] = inoasMinimalGnssStep(0, 4, 2, 0, 0, 1, t, cfg);
    verifyEqual(testCase, [lambda, on, double(state)], [0, 1, 1]);
end
end

function testReactiveAlarmPowersOnWithoutGnssHealth(testCase)
clear inoasMinimalGnssStep inoasFixedGnssStep
cfg = inoasMinimalGnssConfig(1, 3);
for t = 0:96
    [~, on] = inoasMinimalGnssStep(0, 5, 2, 0, 0, 1, t, cfg);
end
verifyFalse(testCase, on);
[lambda, on, state] = inoasMinimalGnssStep(11, 0, NaN, 0, 0, 0, 97, cfg);
verifyEqual(testCase, [lambda, on, double(state)], [0, 1, 1]);
end

function testReactiveAlarmExtendsTracking(testCase)
clear inoasMinimalGnssStep inoasFixedGnssStep
cfg = inoasMinimalGnssConfig(1, 3);
for t = 0:120
    [~, on, state] = inoasMinimalGnssStep(11, 5, 2, 0, 0, 1, t, cfg);
end
verifyTrue(testCase, on);
verifyEqual(testCase, state, uint8(2));
end

function testFixedCalendarIgnoresAlarmAndHealthForPower(testCase)
clear inoasMinimalGnssStep inoasFixedGnssStep
cfg = inoasMinimalGnssConfig(1, 3);
cfg.policy = uint8(1);
for t = 0:800
    [~, on] = inoasMinimalGnssStep(Inf, 0, NaN, 0, 0, 0, t, cfg);
    verifyEqual(testCase, on, mod(t,396) < 96);
end
end

function testFullGnssKeepsPowerButScreensFixes(testCase)
clear inoasMinimalGnssStep inoasFixedGnssStep
cfg = inoasMinimalGnssConfig(1, 3);
cfg.policy = uint8(0);
for t = 0:500
    [lambda, on] = inoasMinimalGnssStep(0, 5, 2, 0, 0, 1, t, cfg);
    verifyTrue(testCase, on);
    verifyEqual(testCase, lambda, t >= 36);
end
[lambda, on, state] = inoasMinimalGnssStep(0, 4, 2, 0, 0, 1, 501, cfg);
verifyEqual(testCase, [lambda, on, double(state)], [0, 1, 1]);
end

function testAcquisitionDelayIsResetAfterQualityLoss(testCase)
clear inoasMinimalGnssStep inoasFixedGnssStep
cfg = inoasMinimalGnssConfig(1, 3);
cfg.policy = uint8(0);
for t = 0:49
    inoasMinimalGnssStep(0, 5, 2, 0, 0, 1, t, cfg);
end
inoasMinimalGnssStep(0, 4, 2, 0, 0, 1, 50, cfg);
for t = 51:86
    [lambda, ~] = inoasMinimalGnssStep(0, 5, 2, 0, 0, 1, t, cfg);
    verifyFalse(testCase, lambda);
end
[lambda, ~] = inoasMinimalGnssStep(0, 5, 2, 0, 0, 1, 87, cfg);
verifyTrue(testCase, lambda);
end

function testReceiverEnergyUsesHeldStates(testCase)
[energy, duty, saving] = inoasReceiverEnergy([0;35;95;395], [1;2;0;1]);
expected = (35*2.34+60*1.8+300*.025)/3600;
verifyEqual(testCase, energy(end), expected, 'AbsTol', 1e-12);
verifyEqual(testCase, duty, 95/395, 'AbsTol', 1e-12);
verifyEqual(testCase, saving, 1-expected/(1.8*395/3600), 'AbsTol', 1e-12);
end

function testContinuousTrackingEnergy(testCase)
[energy, duty, saving] = inoasReceiverEnergy([0;10;100], [2;2;2]);
verifyEqual(testCase, energy(end), .05, 'AbsTol', 1e-12);
verifyEqual(testCase, duty, 1);
verifyEqual(testCase, saving, 0, 'AbsTol', 1e-12);
end

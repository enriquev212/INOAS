function tests = test_gnss_quality_gate
% Function-level regression; does not load or simulate the spacecraft model.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.TestData.oldPath = path;
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'tests', 'legacy'));
testCase.TestData.root = root;
end

function teardownOnce(testCase)
path(testCase.TestData.oldPath);
clear inoasMinimalGnssStep instrument_decision
end

function testReferenceErrorsDoNotGate(testCase)
cfg = inoasMinimalGnssConfig(1, 3);
errors = [0, 5, 5000, -1, NaN, Inf, -Inf];
for hpe = errors
    for vpe = errors
        clear inoasMinimalGnssStep
        [~, ~, ~, quality] = inoasMinimalGnssStep(0, 5, 6, hpe, vpe, 1, 0, cfg);
        verifyTrue(testCase, quality);
    end
end
end

function testReceiverValidityBoundary(testCase)
cfg = inoasMinimalGnssConfig(1, 3);
verifyEqual(testCase, cfg.nsvMin, 5);
cases = [5, 6, 1, 1; 5, eps, 1, 1; 6, 2, 1, 1; ...
    3, 2, 1, 0; 4, 2, 1, 0; ...
    5, 0, 1, 0; 5, -1, 1, 0; 5, 6+eps(6), 1, 0; ...
    5, 2, 0, 0; NaN, 2, 1, 0; Inf, 2, 1, 0; ...
    5, NaN, 1, 0; 5, Inf, 1, 0; 5, 2, NaN, 0; 5, 2, Inf, 0];
for k = 1:size(cases, 1)
    clear inoasMinimalGnssStep
    [~, ~, ~, quality] = inoasMinimalGnssStep(0, cases(k,1), cases(k,2), ...
        NaN, Inf, cases(k,3), 0, cfg);
    verifyEqual(testCase, quality, logical(cases(k,4)));
end
end

function testAcquisitionAndQualityLoss(testCase)
cfg = inoasMinimalGnssConfig(1, 3);
clear inoasMinimalGnssStep
for t = 0:36
    [lambda, on, mode] = inoasMinimalGnssStep(0, 12, 2, NaN, Inf, 1, t, cfg);
    verifyTrue(testCase, on);
    verifyEqual(testCase, lambda, t == 36);
    verifyEqual(testCase, mode, uint8(1 + (t == 36)));
end
[lambda, on, mode] = inoasMinimalGnssStep(0, 4, 2, 0, 0, 1, 37, cfg);
verifyFalse(testCase, lambda);
verifyTrue(testCase, on);
verifyEqual(testCase, mode, uint8(1));
end

function testLegacyReferenceErrorsDoNotGate(testCase)
clear instrument_decision
verifyEqual(testCase, instrument_decision(0, 5, 6, NaN, Inf, 1), 1);
clear instrument_decision
verifyEqual(testCase, instrument_decision(0, 5, 0, 0, 0, 1), 0);
clear instrument_decision
verifyEqual(testCase, instrument_decision(0, 4, 2, 0, 0, 1), 0);
clear instrument_decision
verifyEqual(testCase, instrument_decision(0, NaN, 2, 0, 0, 1), 0);
end

function testSupervisorLocations(testCase)
root = testCase.TestData.root;
verifyEqual(testCase, exist(fullfile(root, 'matlab', 'instrument_decision.m'), 'file'), 0);
verifyEqual(testCase, which('instrument_decision'), ...
    fullfile(root, 'tests', 'legacy', 'instrument_decision.m'));
verifyEqual(testCase, which('inoasMinimalGnssStep'), ...
    fullfile(root, 'matlab', 'inoasMinimalGnssStep.m'));
end

function testFourSatellitesCannotCompleteAcquisition(testCase)
cfg = inoasMinimalGnssConfig(1, 3);
clear inoasMinimalGnssStep
for t = 0:60
    [lambda, on, mode, quality] = inoasMinimalGnssStep(0, 4, 2, 0, 0, 1, t, cfg);
    verifyEqual(testCase, [lambda, on, double(mode), quality], [0, 1, 1, 0]);
end
[lambda, on, mode, quality] = inoasMinimalGnssStep(0, 5, 2, 0, 0, 1, 63, cfg);
verifyEqual(testCase, [lambda, on, double(mode), quality], [1, 1, 2, 1]);
end

function testNoReferenceErrorThresholds(testCase)
cfg = inoasMinimalGnssConfig(1, 3);
verifyFalse(testCase, isfield(cfg, 'hpeMax'));
verifyFalse(testCase, isfield(cfg, 'vpeMax'));
end

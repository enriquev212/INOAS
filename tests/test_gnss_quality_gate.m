function tests = test_gnss_quality_gate
% Function-level regression; does not load or simulate the spacecraft model.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.TestData.oldPath = path;
addpath(fullfile(root, 'matlab'));
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
        [~, ~, ~, quality] = inoasMinimalGnssStep(0, 4, 6, hpe, vpe, 1, 0, cfg);
        verifyTrue(testCase, quality);
    end
end
end

function testReceiverValidityBoundary(testCase)
cfg = inoasMinimalGnssConfig(1, 3);
cases = [4, 6, 1, 1; 4, eps, 1, 1; 3, 2, 1, 0; ...
    4, 0, 1, 0; 4, -1, 1, 0; 4, 6+eps(6), 1, 0; ...
    4, 2, 0, 0; NaN, 2, 1, 0; Inf, 2, 1, 0; ...
    4, NaN, 1, 0; 4, Inf, 1, 0; 4, 2, NaN, 0; 4, 2, Inf, 0];
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
[lambda, on, mode] = inoasMinimalGnssStep(0, 3, 2, 0, 0, 1, 37, cfg);
verifyFalse(testCase, lambda);
verifyTrue(testCase, on);
verifyEqual(testCase, mode, uint8(1));
end

function testLegacyReferenceErrorsDoNotGate(testCase)
clear instrument_decision
verifyEqual(testCase, instrument_decision(0, 4, 6, NaN, Inf, 1), 1);
clear instrument_decision
verifyEqual(testCase, instrument_decision(0, 4, 0, 0, 0, 1), 0);
clear instrument_decision
verifyEqual(testCase, instrument_decision(0, NaN, 2, 0, 0, 1), 0);
end

function testNoReferenceErrorThresholds(testCase)
cfg = inoasMinimalGnssConfig(1, 3);
verifyFalse(testCase, isfield(cfg, 'hpeMax'));
verifyFalse(testCase, isfield(cfg, 'vpeMax'));
end

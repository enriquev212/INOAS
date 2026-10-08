function tests = test_navigation_prediction
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.TestData.oldPath = path;
addpath(fullfile(root, 'matlab'));
end

function teardownOnce(testCase)
path(testCase.TestData.oldPath);
clear predictNavigationCovarianceProfile
end

function testLinearForecastMatchesAuxiliaryRiccati(testCase)
[cfg, F, H, x, P] = fixture();
actual = predictNavigationCovarianceProfile(x, P, 0, (1:12)', zeros(3,1), cfg);
for k = 1:12
    P = F*P*F'+cfg.Qexternal;
    K = (P*H')/(H*P*H'+cfg.Raux);
    P = P-K*(H*P*H'+cfg.Raux)*K';
    verifyEqual(testCase, actual(:,:,k), P, 'AbsTol', 1e-7);
end
end

function testCommonTimesDoNotDependOnMpcGrid(testCase)
[cfg, ~, ~, x, P] = fixture();
fine = predictNavigationCovarianceProfile(x, P, 0, (3:3:36)', zeros(3,1), cfg);
coarse = predictNavigationCovarianceProfile(x, P, 0, (12:12:36)', zeros(3,1), cfg);
verifyEqual(testCase, coarse, fine(:,:,4:4:12), 'AbsTol', 1e-10);
end

function testActuationUncertaintyIncreasesForecast(testCase)
[cfg, ~, ~, x, P] = fixture();
quiet = predictNavigationCovarianceProfile(x, P, 0, [3;12], zeros(3,1), cfg);
active = predictNavigationCovarianceProfile(x, P, 0, [3;12], ones(3,1), cfg);
for k = 1:2
    verifyGreaterThanOrEqual(testCase, min(eig(active(:,:,k)-quiet(:,:,k))), -1e-8);
end
end

function testOrbitalForecastIsFinitePositive(testCase)
[cfg, ~, ~, ~, P] = fixture();
cfg.stateTransitionFcn = @myStateTransitionFcn;
cfg.measurementFcn = @myMeasurementFcn;
cfg.alpha = 1e-3;
x = [7714430;0;0;0;7200;0];
actual = predictNavigationCovarianceProfile(x, P, 0, [3;12;60], zeros(3,1), cfg);
verifyTrue(testCase, all(isfinite(actual), 'all'));
for k = 1:3
    verifyGreaterThanOrEqual(testCase, min(eig(actual(:,:,k))), -1e-8);
end
end

function [cfg, F, H, x, P] = fixture()
F = [eye(3),eye(3);zeros(3),eye(3)];
H = [eye(3),zeros(3)];
x = [100;20;30;1;2;3];
P = diag([25,25,25,.01,.01,.01]);
cfg = struct('sampleTime', 1, 'stateTransitionFcn', @(x,u) F*x+[.5*eye(3);eye(3)]*u, ...
    'measurementFcn', @(x) H*x, 'alpha', .1, 'beta', 2, 'kappa', 0, ...
    'Qexternal', .01*[eye(3)/3,eye(3)/2;eye(3)/2,eye(3)], ...
    'Raux', 4e6*eye(3), 'errorCmd', .1, 'Gcmd', [.5*eye(3);eye(3)]);
end

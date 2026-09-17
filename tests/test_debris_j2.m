function tests = test_debris_j2
% Check the actual debris integrator against an independent Cartesian J2 law.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.TestData.originalPath = path;
addpath(fullfile(root, 'matlab'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testCartesianJ2ForwardStep(testCase)
states = [7.7e6, 0, 0, 0, 7200, 1200; ...
          0, 0, 7.7e6, 7200, 0, 0; ...
          4.444968e6, -4.285885e6, -4.625289e6, -141.37314, 5202.530803, -4957.490129].';
for dt = [1, 12]
    for index = 1:size(states, 2)
        initial = states(:, index);
        expected = cartesianStep(initial, dt);
        filename = [tempname, '.mat'];
        cleanup = onCleanup(@() delete(filename));
        get_debris_trajectory(dt, [initial, expected], [0; dt], 0, ...
            zeros(3, 1), zeros(3, 1), filename);
        actual = load(filename);
        verifyEqual(testCase, actual.x_debris_hist(:, 2), expected, 'AbsTol', 1e-8);
        clear cleanup
    end
end
end

function testIdenticalOrbitsStayIdenticalBothDirections(testCase)
t = (0:300).';
reference = zeros(6, numel(t));
reference(:, 151) = [4.444968e6; -4.285885e6; -4.625289e6; ...
    -141.37314; 5202.530803; -4957.490129];
for index = 152:numel(t)
    reference(:, index) = cartesianStep(reference(:, index - 1), 1);
end
for index = 150:-1:1
    reference(:, index) = cartesianStep(reference(:, index + 1), -1);
end
filename = [tempname, '.mat'];
cleanup = onCleanup(@() delete(filename));
get_debris_trajectory(1, reference, t, 150, zeros(3, 1), zeros(3, 1), filename);
actual = load(filename);
verifyLessThan(testCase, max(vecnorm(actual.x_debris_hist(1:3, :) - reference(1:3, :))), 1e-6);
verifyLessThan(testCase, max(vecnorm(actual.x_debris_hist(4:6, :) - reference(4:6, :))), 1e-8);
end

function testEncounterOffsetIsUnchanged(testCase)
state = [4.444968e6; -4.285885e6; -4.625289e6; -141.37314; 5202.530803; -4957.490129];
position = [50; 0; 0]; velocity = [0; 10; 0];
[rotation, inverse] = referenceFrameTransform(state(1:3), state(4:6));
omega = [0; 0; norm(cross(state(1:3), state(4:6))) / norm(state(1:3))^2];
filename = [tempname, '.mat'];
cleanup = onCleanup(@() delete(filename));
get_debris_trajectory(1, [state, cartesianStep(state, 1)], [0; 1], 0, position, velocity, filename);
actual = load(filename);
expected = state + [inverse * position; inverse * (velocity + cross(omega, position))];
verifyEqual(testCase, actual.x_debris_hist(:, 1), expected, 'AbsTol', 1e-8);
verifyEqual(testCase, rotation * (actual.x_debris_hist(1:3, 1) - state(1:3)), position, 'AbsTol', 1e-8);
end

function next = cartesianStep(x, dt)
a = cartesianDerivative(x);
b = cartesianDerivative(x + dt * a / 2);
c = cartesianDerivative(x + dt * b / 2);
d = cartesianDerivative(x + dt * c);
next = x + dt * (a + 2*b + 2*c + d) / 6;
end

function dx = cartesianDerivative(x)
r = x(1:3); radius = norm(r); z2 = (r(3) / radius)^2;
mu = 3.986004418e14; earthRadius = 6378137; j2 = 1.08262668e-3;
acceleration = -mu * r / radius^3 + 1.5 * j2 * mu * earthRadius^2 / radius^5 * ...
    [r(1) * (5*z2 - 1); r(2) * (5*z2 - 1); r(3) * (5*z2 - 3)];
dx = [x(4:6); acceleration];
end

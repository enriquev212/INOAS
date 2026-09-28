function tests = test_default_gnss_dataset
% Dataset/loading regression; never initializes or simulates the spacecraft.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
testCase.TestData.root = root;
testCase.TestData.oldPath = path;
testCase.TestData.oldRng = rng;
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'tests'));
testCase.TestData.filename = 'full_perturb_POS_s6a_Y24D011_fixed.dat';
testCase.TestData.legacy = 'cov_perturb_POS_s6a_Y24D011_fixed.dat';
testCase.TestData.data = readmatrix(inoas_data_file(testCase.TestData.filename), ...
    'CommentStyle', '#');
names = {'ts_gnss_sol', 'ts_gnss_nsv', 'ts_gnss_hpe', 'ts_gnss_vpe', ...
    'ts_gnss_pdop', 'Ts', 'gnssCovarianceFile'};
saved = struct();
for k = 1:numel(names)
    name = names{k};
    saved.(name).exists = evalin('base', sprintf('exist(''%s'', ''var'')', name));
    if saved.(name).exists
        saved.(name).value = evalin('base', name);
    end
end
testCase.TestData.savedBase = saved;
testCase.TestData.modelWasLoaded = bdIsLoaded('inoas_model');
end

function teardownOnce(testCase)
saved = testCase.TestData.savedBase;
names = fieldnames(saved);
for k = 1:numel(names)
    name = names{k};
    if saved.(name).exists
        assignin('base', name, saved.(name).value);
    else
        evalin('base', ['clear ' name]);
    end
end
if ~testCase.TestData.modelWasLoaded && bdIsLoaded('inoas_model')
    close_system('inoas_model', 0);
end
rng(testCase.TestData.oldRng);
path(testCase.TestData.oldPath);
end

function testDatasetShapeAndCadence(testCase)
data = testCase.TestData.data;
verifySize(testCase, data, [8640, 17]);
verifyEqual(testCase, data(:, 1), (0:10:86390)');
verifyTrue(testCase, all(isfinite(data), 'all'));
end

function testOnlyDocumentedChangesFromLegacy(testCase)
data = testCase.TestData.data;
legacy = readmatrix(inoas_data_file(testCase.TestData.legacy), 'CommentStyle', '#');
changed = data ~= legacy;
expectedRows = (data(:, 1) >= 2000 & data(:, 1) < 2260) | ...
    (data(:, 1) >= 3500 & data(:, 1) < 3760);
verifyEqual(testCase, any(changed, 2), expectedRows);
verifyEqual(testCase, nnz(any(changed, 2)), 52);
allowed = false(size(data));
allowed(data(:, 1) >= 2000 & data(:, 1) < 2260, [8, 9, 15:17]) = true;
allowed(data(:, 1) >= 3500 & data(:, 1) < 3760, 15:17) = true;
verifyFalse(testCase, any(changed & ~allowed, 'all'));
end

function testDefaultQualityRejectionIntervals(testCase)
load_gnss_quality_signals();
sol = evalin('base', 'ts_gnss_sol');
nsv = evalin('base', 'ts_gnss_nsv');
pdop = evalin('base', 'ts_gnss_pdop');
t = sol.Time;
quality = isfinite(sol.Data) & isfinite(nsv.Data) & isfinite(pdop.Data) & ...
    sol.Data >= 0.5 & nsv.Data >= 5 & pdop.Data > 0 & pdop.Data <= 6;
expected = ~((t >= 700 & t < 980) | (t >= 2000 & t < 2260) | ...
    (t >= 3500 & t < 3550));
arc = t <= 6743;
verifyEqual(testCase, quality(arc), expected(arc));
end

function testSensorProfileDefault(testCase)
profile = load_gnss_sensor_profile();
verifyEqual(testCase, profile.filename, string(inoas_data_file(testCase.TestData.filename)));
verifyEqual(testCase, profile.sample_time, 10);
end

function testInitialQualityPaddingRequiresFiveSatellites(testCase)
data = testCase.TestData.data(1:4, :);
data(:, 7) = 1;
data(:, 9) = [0; 4; 5; 6];
data(:, 17) = 2;
filename = [tempname '.dat'];
cleanup = onCleanup(@() delete(filename)); %#ok<NASGU>
writetable(array2table(data), filename, 'FileType', 'text', 'Delimiter', ' ');
load_gnss_quality_signals(filename);
nsv = evalin('base', 'ts_gnss_nsv');
verifyEqual(testCase, nsv.Data, [5; 5; 5; 6]);
end

function testVersionedInputsKeepSameReceiverDecisions(testCase)
filenames = {testCase.TestData.filename, testCase.TestData.legacy, ...
    'perturb_POS_s6a_Y24D011.dat'};
for k = 1:numel(filenames)
    report = compare_gnss_minimum_replay(inoas_data_file(filenames{k}), 6743);
    verifyEqual(testCase, report.recordsWithFourUsedSatellites, 0);
    verifyEqual(testCase, report.qualityDifferences24h, 0);
    for j = 1:numel(report.receiverCases)
        verifyEqual(testCase, report.receiverCases(j).differingSteps, zeros(1, 4));
        verifyEqual(testCase, report.receiverCases(j).acceptedFixDifferences, 0);
    end
end
end

function testSensorWorkspaceDefault(testCase)
[sensor, profile] = prepare_gnss_sensor_workspace('StopTime', 0);
verifyEqual(testCase, profile.filename, string(inoas_data_file(testCase.TestData.filename)));
verifyEqual(testCase, sensor.sample_time, 3);
end

function testInitializerSelectsS2(testCase)
source = fileread(fullfile(testCase.TestData.root, 'initialize_inoas_simulation.m'));
expected = ['gnssCovarianceFile = inoas_data_file("' testCase.TestData.filename '");'];
verifyTrue(testCase, contains(source, expected));
end

function testModelCallbackUsesSelectedInput(testCase)
load_system(fullfile(testCase.TestData.root, 'models', 'inoas_model.slx'));
callback = get_param('inoas_model', 'InitFcn');
evalin('base', 'clear gnssCovarianceFile');
evalin('base', callback);
nsv = evalin('base', 'ts_gnss_nsv');
data = testCase.TestData.data;
verifyEqual(testCase, nsv.Data(nsv.Time == 2000), data(data(:, 1) == 2000, 9));

legacyPath = inoas_data_file(testCase.TestData.legacy);
legacy = readmatrix(legacyPath, 'CommentStyle', '#');
assignin('base', 'gnssCovarianceFile', legacyPath);
evalin('base', callback);
nsv = evalin('base', 'ts_gnss_nsv');
verifyEqual(testCase, nsv.Data(nsv.Time == 2000), legacy(legacy(:, 1) == 2000, 9));

assignin('base', 'gnssCovarianceFile', inoas_data_file(testCase.TestData.filename));
evalin('base', callback);
nsv = evalin('base', 'ts_gnss_nsv');
verifyEqual(testCase, nsv.Data(nsv.Time == 2000), data(data(:, 1) == 2000, 9));
end

function tests = test_example_launchers
tests = functiontests(localfunctions);
end

function setup(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
% Isolated stubs check launcher paths without opening Simulink or running a case.
folder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
project = fullfile(folder.Folder, 'project');
mkdir(fullfile(project, 'examples'));
names = {'open_inoas_fast.m', 'open_inoas_debris_demo.m', 'run_navigation_trial.m'};
for k = 1:numel(names)
    copyfile(fullfile(root, 'examples', names{k}), fullfile(project, 'examples', names{k}));
end
writeStub(fullfile(project, 'open_inoas_model.m'), sprintf([ ...
    'setappdata(0, ''inoasExampleStopTime'', simulationStopTime);\n' ...
    'setappdata(0, ''inoasExampleRoot'', fileparts(mfilename(''fullpath'')));\n']));
writeStub(fullfile(project, 'set_param.m'), sprintf([ ...
    'function set_param(varargin)\n' ...
    'setappdata(0, ''inoasExampleParameters'', varargin);\n' ...
    'end\n']));
writeStub(fullfile(project, 'run_inoas_case.m'), sprintf([ ...
    'function outputDir = run_inoas_case(varargin)\n' ...
    'setappdata(0, ''inoasExampleCase'', varargin);\n' ...
    'outputDir = fullfile(fileparts(mfilename(''fullpath'')), ''results'');\n' ...
    'end\n']));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture(project));
outside = fullfile(folder.Folder, 'outside');
mkdir(outside);
testCase.applyFixture(matlab.unittest.fixtures.CurrentFolderFixture(outside));
testCase.TestData.project = project;
testCase.TestData.names = {'simulationStopTime', 'outputDir'};
for k = 1:numel(testCase.TestData.names)
    name = testCase.TestData.names{k};
    present = evalin('base', sprintf('exist(''%s'', ''var'')', name)) ~= 0;
    testCase.TestData.present(k) = present;
    if present
        testCase.TestData.values{k} = evalin('base', name);
    end
end
end

function teardown(testCase)
for k = 1:numel(testCase.TestData.names)
    name = testCase.TestData.names{k};
    if testCase.TestData.present(k)
        assignin('base', name, testCase.TestData.values{k});
    else
        evalin('base', ['clear ' name]);
    end
end
names = {'inoasExampleStopTime', 'inoasExampleRoot', ...
    'inoasExampleParameters', 'inoasExampleCase'};
for k = 1:numel(names)
    if isappdata(0, names{k}), rmappdata(0, names{k}); end
end
clear run_inoas_case set_param
end

function testFastLauncherOutsideRepository(testCase)
runExample(testCase, 'open_inoas_fast.m');
verifyEqual(testCase, getappdata(0, 'inoasExampleRoot'), testCase.TestData.project);
verifyEqual(testCase, getappdata(0, 'inoasExampleStopTime'), 120);
verifyEqual(testCase, getappdata(0, 'inoasExampleParameters'), ...
    {"inoas_model", "StopTime", "120"});
end

function testEncounterLauncherOutsideRepository(testCase)
runExample(testCase, 'open_inoas_debris_demo.m');
verifyEqual(testCase, getappdata(0, 'inoasExampleRoot'), testCase.TestData.project);
verifyEqual(testCase, getappdata(0, 'inoasExampleStopTime'), 1800);
verifyEqual(testCase, getappdata(0, 'inoasExampleParameters'), ...
    {"inoas_model", "StopTime", "1800"});
end

function testNavigationLauncherOutsideRepository(testCase)
runExample(testCase, 'run_navigation_trial.m');
verifyEqual(testCase, getappdata(0, 'inoasExampleCase'), ...
    {'reactive', 'adaptive', 'StopTime', 1000});
verifyEqual(testCase, evalin('base', 'outputDir'), ...
    fullfile(testCase.TestData.project, 'results'));
end

function runExample(testCase, name)
script = fullfile(testCase.TestData.project, 'examples', name);
evalin('base', sprintf('run(''%s'');', strrep(script, '''', '''''')));
end

function writeStub(file, source)
fid = fopen(file, 'w');
assert(fid >= 0, 'Cannot write test fixture.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', source);
end

function tests = test_exporters
tests = functiontests(localfunctions);
end

function setup(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'matlab'), fullfile(root, 'tools', 'visualization'));
names = {'out','time_ref','ref_ts_x','ref_ts_y','ref_ts_z','ref_ts_vx','ref_ts_vy','ref_ts_vz', ...
    'x_debris_hist','rk_debris','t_debris_ref','mpc_dsafe_log_time','mpc_dsafe_log_first', ...
    'mpc_dsafe_log_max','ts_gnss_nsv','ts_gnss_pdop','ts_gnss_hpe','ts_gnss_vpe', ...
    'ts_gnss_sol','dsafe0','u_max','t_debris','Np','h','gnss_sample_time'};
snapshot = cell(size(names));
present = false(size(names));
for k = 1:numel(names)
    present(k) = evalin('base', sprintf('exist(''%s'',''var'')', names{k})) ~= 0;
    if present(k), snapshot{k} = evalin('base', names{k}); end
    assignin('base', names{k}, []);
end
testCase.TestData.names = names;
testCase.TestData.snapshot = snapshot;
testCase.TestData.present = present;
folder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
testCase.TestData.folder = folder.Folder;

t = (0:5:60)';
te = (0:10:60)';
truth = repmat([7e6, 0, 0], numel(t), 1);
logs = Simulink.SimulationData.Dataset;
logs = addSignal(logs, 'truth_position_eci', truth, t);
logs = addSignal(logs, 'Estimated_Pos_x', repmat([7e6+1,0,0],numel(te),1), te);
logs = addSignal(logs, 'u_MPC', repmat([.1,0,0],numel(te),1), te);
logs = addSignal(logs, 'applied_acceleration_eci', repmat([.2,0,0],numel(t),1), t);
logs = addSignal(logs, 'receiver_mode', [1;2;0;0], [0;10;30;60]);
out = struct('logsout', logs);
assignin('base','out',out);
assignin('base','time_ref',t);
refNames = {'ref_ts_x','ref_ts_y','ref_ts_z','ref_ts_vx','ref_ts_vy','ref_ts_vz'};
values = [7e6,0,0,0,7000,0];
for k = 1:numel(refNames)
    assignin('base', refNames{k}, timeseries(values(k)*ones(size(t)),t));
end
assignin('base','dsafe0',295);
end

function teardown(testCase)
for k = 1:numel(testCase.TestData.names)
    name = testCase.TestData.names{k};
    if testCase.TestData.present(k)
        assignin('base', name, testCase.TestData.snapshot{k});
    else
        evalin('base', ['clear ' name]);
    end
end
end

function testCsvUsesStateEnergyAndAppliedAcceleration(testCase)
export_campaign_csv(testCase.TestData.folder);
metrics = readtable(fullfile(testCase.TestData.folder,'metrics.csv'));
metric = @(name) metrics.value(strcmp(metrics.metric,name));
verifyEqual(testCase, metric('receiver_energy_Wh'), 60.15/3600, 'AbsTol',1e-12);
verifyEqual(testCase, metric('receiver_powered_fraction'), .5, 'AbsTol',1e-12);
verifyEqual(testCase, metric('final_delta_v_mps'), 12, 'AbsTol',1e-12);
verifyEqual(testCase, metric('final_commanded_delta_v_mps'), 6, 'AbsTol',1e-12);
applied = readtable(fullfile(testCase.TestData.folder,'applied_control.csv'));
verifyEqual(testCase, applied.applied_delta_v_mps(end), 12, 'AbsTol',1e-12);
ts = readtable(fullfile(testCase.TestData.folder,'timeseries.csv'));
verifyEqual(testCase, unique(ts.keepout_distance_m), 150);
verifyEqual(testCase, unique(ts.safe_radius_m), 295);
end

function testCompactMatPreservesDifferentSamplingGrids(testCase)
file = fullfile(testCase.TestData.folder,'visuals.mat');
export_visualization_data(file);
data = load(file);
verifyEqual(testCase, size(data.estimated_eci_m,1), numel(data.estimate_time_s));
verifyNotEqual(testCase, numel(data.estimate_time_s), numel(data.time_s));
verifyEqual(testCase, data.receiver_energy_Wh(end), 60.15/3600, 'AbsTol',1e-12);
verifyEqual(testCase, data.applied_eci_mps2(:,1), .2*ones(size(data.applied_time_s)));
end

function testAmbiguousLogNameIsRejected(testCase)
out = evalin('base','out');
logs = out.logsout;
logs = addSignal(logs,'truth_position_eci',zeros(3,3),[0;30;60]);
out.logsout = logs;
assignin('base','out',out);
verifyError(testCase,@() export_campaign_csv(testCase.TestData.folder),'INOAS:AmbiguousLogSignal');
end

function logs = addSignal(logs, name, data, time)
signal = Simulink.SimulationData.Signal;
signal.Name = name;
signal.Values = timeseries(data,time);
logs = logs.addElement(signal,name);
end

function outputDir = run_navigation_smoke_test(reuseBaseline)
%RUN_NAVIGATION_SMOKE_TEST Compile and simulate in an isolated temporary copy.
% Existing reference/debris MAT files and user simulation outputs are untouched.
if nargin == 0
    reuseBaseline = false;
end
repoRoot = fileparts(fileparts(mfilename('fullpath')));
previousPath = path;
previousDir = pwd;
names = {'mpcTuneConfig','skipBatchClear'};
prefs = cell(size(names));
present = false(size(names));
for k=1:numel(names)
    present(k) = ispref('inoas',names{k});
    if present(k)
        prefs{k} = getpref('inoas',names{k});
    end
end
cleanup = onCleanup(@() restoreContext(previousPath,previousDir,names,present,prefs)); %#ok<NASGU>
sandbox = tempname;
mkdir(sandbox);
for folder = ["matlab","models","data"]
    copyfile(fullfile(repoRoot,folder),fullfile(sandbox,folder));
end
copyfile(fullfile(repoRoot,'initialize_inoas_simulation.m'),sandbox);
mkdir(fullfile(sandbox,'tools'));
addpath(fullfile(repoRoot,'tools'),fullfile(repoRoot,'tools','visualization'));
cd(sandbox);
outputDir = fullfile(repoRoot,'results','navigation_smoke');
fprintf('Isolated smoke-test workspace: %s\n',sandbox);

if bdIsLoaded('inoas_model')
    error('INOAS:ModelAlreadyLoaded','Close inoas_model before running the isolated test.');
end
setpref('inoas','skipBatchClear',true);
assignin('base','modelName','inoas_model');
assignin('base','simulationStopTime',9);
initPath = strrep(fullfile(sandbox,'initialize_inoas_simulation.m'),'''','''''');
evalin('base',sprintf('run(''%s'');',initPath));
assignin('base','mpcQuiet',true);
evalin('base','rng(7);');
load_system(fullfile(sandbox,'models','inoas_model.slx'));
modelCleanup = onCleanup(@() close_system('inoas_model',0)); %#ok<NASGU>
set_param('inoas_model','StopTime','9');
configure_navigation_prediction('inoas_model'); % also checks idempotence
if ~reuseBaseline
    set_param('inoas_model','SimulationCommand','update');
end
actual = get_param('inoas_model/Mux','PortHandles');
expected = get_param('inoas_model/Kalman Filter','PortHandles');
for k=1:2
    line = get_param(actual.Inport(k),'Line');
    assert(get_param(line,'SrcPortHandle')==expected.Outport(k));
end
if reuseBaseline
    saved = load(fullfile(outputDir,'smoke_output.mat'),'out');
    savedLog = load(fullfile(outputDir,'navigation_prediction.mat'),'predictionLog');
    out = saved.out;
    assignin('base','out',out);
    assignin('base','mpc_navigation_prediction_log',savedLog.predictionLog);
else
    evalin('base','clear MPC_INOAS instrument_decision; out = sim(''inoas_model'');');
    out = evalin('base','out');
    save(fullfile(outputDir,'smoke_output.mat'),'out');
end
export_navigation_prediction_csv(outputDir,out);
export_visualization_data(fullfile(outputDir,'raw_visualization_data.mat'));
export_campaign_csv(outputDir);
forecast = evalin('base','mpc_navigation_prediction_log');
assert(numel(forecast.origin_time)>=3);
assert(all(abs(forecast.target_time(1,:).'-forecast.origin_time-3)<1e-9));
assert(all(isfinite(forecast.P_eci),'all'));
comparison = readtable(fullfile(outputDir,'navigation_prediction.csv'));
assert(any(isfinite(comparison.realized_sigma_max_pos_m)));
assert(all(isnan(comparison.realized_sigma_max_pos_m(comparison.target_time_s>9))));
navigation = readtable(fullfile(outputDir,'navigation_uncertainty.csv'));
assert(all(isfinite(navigation.sigma_max_pos_m)));
assert(all(isfinite(navigation.lambda_effective)));
save(fullfile(outputDir,'smoke_output.mat'),'out');
fprintf('Default Np=125, h=3 navigation integration and CSV export passed.\n');

% Exercise the supervisor transition at t=60 without a long optimization run.
set_param('inoas_model','StopTime','66');
assignin('base','simulationStopTime',66);
setpref('inoas','skipBatchClear',true);
setpref('inoas','mpcTuneConfig',struct('Np',5, ...
    'covariancePredictionModeMpc',"navigation_scheduled"));
evalin('base',sprintf('run(''%s'');',initPath));
assignin('base','mpcQuiet',true);
evalin('base','clear MPC_INOAS instrument_decision; rng(7); out = sim(''inoas_model'');');
out = evalin('base','out');
export_navigation_prediction_csv(fullfile(outputDir,'scheduled'),out);
status = out.logsout.getElement('navigation_status').Values;
lambda = out.logsout.getElement('lambda').Values;
statusData = squeeze(status.Data);
if size(statusData,1)==3 && size(statusData,2)==numel(status.Time)
    statusData = statusData.';
end
lambdaAtStatus = interp1(lambda.Time,double(lambda.Data(:)),status.Time,'previous');
assert(all(statusData(:,1)==lambdaAtStatus));
assert(any(lambda.Data(:)==0), 'The scheduled test must exercise a GNSS-off transition.');
save(fullfile(outputDir,'scheduled','smoke_output.mat'),'out');
fprintf('Scheduled mode, delayed status, GNSS-off transition, and CSV export passed.\n');
end

function restoreContext(previousPath,previousDir,names,present,prefs)
path(previousPath);
cd(previousDir);
for k=1:numel(names)
    if present(k)
        setpref('inoas',names{k},prefs{k});
    elseif ispref('inoas',names{k})
        rmpref('inoas',names{k});
    end
end
end

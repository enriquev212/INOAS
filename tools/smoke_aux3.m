function smoke_aux3(stopTime)
%SMOKE_AUX3 Short closed-loop run after removing the altitude channel.
% Checks the measurement dimensions, that the model runs end to end, and that
% the auxiliary score settles around 3 (chi-square with three degrees of
% freedom) instead of around 4.
if nargin < 1, stopTime = 600; end
root = fileparts(fileparts(mfilename('fullpath')));
cd(root);
addpath(root, fullfile(root, 'matlab'), fullfile(root, 'tools'));

assert(isequal(size(myMeasurementFcn([7.7e6; 0; 0; 0; 7.2e3; 0])), [3 1]), ...
    'myMeasurementFcn must return 3 elements.');

assignin('base', 'simulationStopTime', stopTime);
evalin('base', 'initialize_inoas_simulation');
R = evalin('base', 'R_matrix');
assert(isequal(size(R), [3 3]), 'R_matrix must be 3x3.');
fprintf('R_matrix = diag(%.0f %.0f %.0f)\n', R(1,1), R(2,2), R(3,3));

model = 'inoas_model';
if ~bdIsLoaded(model), load_system(fullfile(root, 'models', [model '.slx'])); end
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
set_param(model, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'ReturnWorkspaceOutputs', 'on', 'StopTime', num2str(stopTime));

% Log the auxiliary score and the auxiliary measurement itself.
score = Simulink.ID.getHandle('inoas_model:400');   % Pseudo NIS subsystem
sensor = Simulink.ID.getHandle('inoas_model:139');  % Sensor_Simulation_Model
for pair = {score, 'aux_score'; sensor, 'aux_measurement'}'
    ports = get_param(pair{1}, 'PortHandles');
    set_param(ports.Outport(1), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', pair{2});
end

out = sim(model);
logs = out.logsout;
z = logs.getElement('aux_measurement').Values;
s = logs.getElement('aux_score').Values;
width = size(z.Data, 2) * size(z.Data, 3);
fprintf('aux measurement width: %d (expected 3)\n', width);
after = s.Time >= 135;
fprintf('aux score after 135 s: mean %.2f, median %.2f (expected about 3)\n', ...
    mean(s.Data(after)), median(s.Data(after)));
fprintf('smoke_aux3: completed %g s of closed loop.\n', stopTime);
end

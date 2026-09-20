function apply_aux3_removal(root)
%APPLY_AUX3_REMOVAL Drop the derived-altitude channel from the auxiliary sensor.
%
% The auxiliary measurement was z = [x y z h] with h = |r| - R_earth computed
% from the same position, so the fourth channel carried no independent
% information while R was diagonal. Two edits are needed:
%
%   1. Sensor_Simulation_Model (inside Kalman Filter) outputs the 3-component
%      noisy position instead of concatenating the noisy altitude:
%        before:  x -> Sum <- Magnetometer Noise ---> Concatenate -> Z_M
%                 x -> altimetry -> Sum1 <- Altimetry Noise ------^
%        after:   x -> Sum <- Magnetometer Noise -------------> Z_M
%   2. The synthetic fault injector (inyectar_fallo) emits 3-vectors, otherwise
%      its sum with the measurement fails the dimension check.
%
% Blocks are addressed by SID and lines by port handle, so the newline inside
% the 'Vector\nConcatenate' block name cannot break the edit. Each step is
% independent and idempotent.
if nargin < 1
    root = fileparts(fileparts(mfilename('fullpath')));
end
model = 'inoas_model';
wasLoaded = bdIsLoaded(model);
if ~wasLoaded
    load_system(fullfile(root, 'models', [model '.slx']));
end
closer = onCleanup(@() closeIfOpened(model, wasLoaded)); %#ok<NASGU>

changed = rewireSensorModel(model);
changed = patchFaultInjector(model) || changed;
changed = rewireErrorMonitor(model) || changed;

if changed
    save_system(model);
    fprintf('apply_aux3_removal: model saved with a 3-dimensional auxiliary measurement.\n');
else
    fprintf('apply_aux3_removal: nothing to do, model already converted.\n');
end
end


function changed = rewireSensorModel(model)
changed = false;
concat = findBySid(model, 137);      % Vector Concatenate
if isempty(concat)
    return
end
sumPos = findBySid(model, 134);      % Sum: truth position + magnetometer noise
outPort = findBySid(model, 141);     % Outport Z_M
altFcn = findBySid(model, 136);      % MATLAB Function: altimetry
altSum = findBySid(model, 135);      % Sum1: altitude + altimetry noise
altNoise = findBySid(model, 133);    % Altimetry Noise
noise = findBySid(model, 132);       % Magnetometer Noise
inX = findBySid(model, 140);         % Inport x (true position)
assert(~isempty(sumPos) && ~isempty(outPort) && ~isempty(noise) && ~isempty(inX), ...
    'INOAS:Aux3Layout', 'Unexpected Sensor_Simulation_Model layout.');
parent = get_param(concat, 'Parent');

for h = [concat, altSum, altFcn, altNoise, outPort, sumPos]
    if isempty(h) || ~ishandle(h), continue; end
    ports = get_param(h, 'PortHandles');
    for p = [ports.Inport(:); ports.Outport(:)].'
        line = get_param(p, 'Line');
        if line ~= -1
            delete_line(line);
        end
    end
end
for h = [concat, altSum, altFcn, altNoise]
    if ~isempty(h) && ishandle(h)
        delete_block(h);
    end
end

sumPorts = get_param(sumPos, 'PortHandles');
add_line(parent, get_param(noise, 'PortHandles').Outport(1), sumPorts.Inport(1), 'autorouting', 'on');
add_line(parent, get_param(inX, 'PortHandles').Outport(1), sumPorts.Inport(2), 'autorouting', 'on');
add_line(parent, sumPorts.Outport(1), get_param(outPort, 'PortHandles').Inport(1), 'autorouting', 'on');
changed = true;
fprintf('apply_aux3_removal: altitude channel removed from Sensor_Simulation_Model.\n');
end


function changed = rewireErrorMonitor(model)
% The Error_Z_Kalman scope compares the measurement with the predicted
% auxiliary vector, which concatenated the estimated altitude. Drop that branch
% and feed the estimated position straight into the difference.
changed = false;
concat = findBySid(model, 398);      % Vector Concatenate: [estimated pos; estimated altitude]
if isempty(concat)
    return
end
altFcn = findBySid(model, 397);      % MATLAB Function1: altitude of the estimate
diff = findBySid(model, 396);        % Sum2: measurement minus prediction
estimate = findBySid(model, 142);    % Demux of the UKF state
assert(~isempty(diff) && ~isempty(estimate), 'INOAS:Aux3Monitor', ...
    'Unexpected error-monitor layout.');
parent = get_param(concat, 'Parent');

% Delete only the branches that reach the blocks being removed, so the other
% consumers of the estimate (outport and hold) keep their connections.
for h = [concat, altFcn]
    if isempty(h) || ~ishandle(h), continue; end
    ports = get_param(h, 'PortHandles');
    for p = [ports.Inport(:); ports.Outport(:)].'
        line = get_param(p, 'Line');
        if line ~= -1
            delete_line(line);
        end
    end
end
for h = [concat, altFcn]
    if ~isempty(h) && ishandle(h)
        delete_block(h);
    end
end
add_line(parent, get_param(estimate, 'PortHandles').Outport(1), ...
    get_param(diff, 'PortHandles').Inport(2), 'autorouting', 'on');
changed = true;
fprintf('apply_aux3_removal: error monitor now compares 3-component vectors.\n');
end


function changed = patchFaultInjector(model)
changed = false;
charts = sfroot().find('-isa', 'Stateflow.EMChart');
for k = 1:numel(charts)
    chart = charts(k);
    if ~startsWith(chart.Path, model) || ~contains(chart.Script, 'inyectar_fallo')
        continue
    end
    updated = strrep(chart.Script, '[500; 500; 500;0]', '[500; 500; 500]');
    updated = strrep(updated, '[500; 500; 500; 0]', '[500; 500; 500]');
    updated = strrep(updated, '[0; 0; 0; 0]', '[0; 0; 0]');
    if ~strcmp(updated, chart.Script)
        chart.Script = updated;
        changed = true;
        fprintf('apply_aux3_removal: fault injector in %s reduced to 3 elements.\n', chart.Path);
    end
end
end


function h = findBySid(model, sid)
try
    h = Simulink.ID.getHandle(sprintf('%s:%d', model, sid));
catch
    h = [];
end
end


function closeIfOpened(model, wasLoaded)
if ~wasLoaded && bdIsLoaded(model)
    close_system(model, 0);
end
end

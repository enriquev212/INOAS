function apply_instrument_duty_cycle_fix(model)
%APPLY_INSTRUMENT_DUTY_CYCLE_FIX Update the embedded decision FSM.
%
% The MATLAB Function block stores its own copy of instrument_decision.m.
% Editing the .m file is not enough; this helper pushes the same code into
% the Simulink chart and saves the model.

if nargin < 1 || strlength(string(model)) == 0
    model = "MPCcontrolledSpacecraft_plant_gnss_kalman_decision";
else
    model = string(model);
end

decisionFile = fullfile(pwd, "instrument_decision.m");
if ~isfile(decisionFile)
    error("Cannot find %s. Run from the INOAS_v4 folder.", decisionFile);
end

load_system(model);

rt = sfroot;
charts = rt.find("-isa", "Stateflow.EMChart");
target = [];
for idx = 1:numel(charts)
    if contains(string(charts(idx).Path), model + "/INSTRUMENT DECISION1/Instrument Decision FSM")
        target = charts(idx);
        break;
    end
end

if isempty(target)
    error("Could not find embedded Instrument Decision FSM chart in %s.", model);
end

target.Script = fileread(decisionFile);
save_system(model);

fprintf("Applied real GNSS/Kalman duty-cycle logic to %s.\n", model);

end

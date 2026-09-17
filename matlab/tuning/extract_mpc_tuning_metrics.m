function metrics = extract_mpc_tuning_metrics(out, stopTime)
%EXTRACT_MPC_TUNING_METRICS Compute fuel, safety and tracking metrics.
%
% Delta-V is the time integral of the Euclidean norm of the applied inertial
% acceleration. This is rotation-invariant, so no LVLH transform is needed.
% The robust margin compares truth/debris separation at t+h with the first
% covariance-inflated safety radius predicted by the MPC at time t.

arguments
    out
    stopTime (1,1) double {mustBePositive}
end

logsout = out.logsout;

truthLog = logsout.get('X_perfect_sensor').Values;
tTruth = double(truthLog.Time(:));
xTruth = squeeze(truthLog.Data);
if size(xTruth, 2) ~= 6 && size(xTruth, 1) == 6
    xTruth = xTruth.';
end
xTruth = double(xTruth);

uLog = get_logsout_signal(logsout, {"u_MPC", "u_discret"});
tU = double(uLog.Time(:));
u = squeeze(uLog.Data);
if size(u, 2) ~= 3 && size(u, 1) == 3
    u = u.';
end
u = double(u);

% Remove duplicate timestamps before integration/interpolation.
[tU, iu] = unique(tU, 'stable');
u = u(iu, :);
[tTruth, ix] = unique(tTruth, 'stable');
xTruth = xTruth(ix, :);

% Reference trajectory from the initialized scenario.
tRef = evalin('base', 'time_ref(:)');
refData = [ ...
    evalin('base', 'ref_ts_x.Data(:)'), ...
    evalin('base', 'ref_ts_y.Data(:)'), ...
    evalin('base', 'ref_ts_z.Data(:)')];
rRef = interp1(tRef, refData, tTruth, 'pchip', 'extrap');
posErr = vecnorm(xTruth(:, 1:3) - rRef, 2, 2);

% Debris truth trajectory uses the same reference timeline.
xDebris = evalin('base', 'x_debris_hist');
if size(xDebris, 1) ~= 6 && size(xDebris, 2) == 6
    xDebris = xDebris.';
end
tDebrisHist = tRef(1:size(xDebris, 2));
rDebris = interp1(tDebrisHist, xDebris(1:3, :).', tTruth, 'pchip', 'extrap');
separation = vecnorm(xTruth(:, 1:3) - rDebris, 2, 2);

% Total applied delta-V.
uNorm = vecnorm(u, 2, 2);
deltaV = trapz(tU, uNorm);

% Robust safety margin. dsafe_first(t) is the first predicted node at t+h.
h = evalin('base', 'h');
if evalin('base', 'exist(''mpc_dsafe_log_time'', ''var'')') && ...
        evalin('base', 'exist(''mpc_dsafe_log_first'', ''var'')')
    tSafe = evalin('base', 'mpc_dsafe_log_time(:)') + h;
    dSafe = evalin('base', 'mpc_dsafe_log_first(:)');
    valid = isfinite(tSafe) & isfinite(dSafe) & tSafe >= tTruth(1) & tSafe <= tTruth(end);
    tSafe = tSafe(valid);
    dSafe = dSafe(valid);
    if isempty(tSafe)
        minRobustMargin = -inf;
    else
        sepAtSafe = interp1(tTruth, separation, tSafe, 'linear', 'extrap');
        minRobustMargin = min(sepAtSafe - dSafe);
    end
else
    % A missing dynamic-safety log means safety could not be verified.
    minRobustMargin = -inf;
end

% Recovery metric: RMS tracking error beginning 300 s after closest approach.
tEncounter = evalin('base', 't_debris');
postMask = tTruth >= (tEncounter + 300);
if any(postMask)
    postRms = sqrt(mean(posErr(postMask).^2));
else
    postRms = posErr(end);
end

controlLimit = evalin('base', 'u_max');
finiteSignals = all(isfinite(xTruth), 'all') && all(isfinite(u), 'all') && ...
                all(isfinite(posErr)) && all(isfinite(separation));

metrics = struct();
metrics.deltaV = deltaV;
metrics.minSeparation = min(separation);
metrics.minRobustMargin = minRobustMargin;
metrics.finalPositionError = posErr(end);
metrics.postEncounterRmsError = postRms;
metrics.maxPositionError = max(posErr);
metrics.maxControlAbs = max(abs(u), [], 'all');
metrics.controlLimit = controlLimit;
metrics.finiteSignals = finiteSignals;
metrics.stopTime = stopTime;
end

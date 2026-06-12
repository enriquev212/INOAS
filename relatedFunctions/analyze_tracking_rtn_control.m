function audit = analyze_tracking_rtn_control(stopTime)
%ANALYZE_TRACKING_RTN_CONTROL Check tracking RTN error against MPC control.
%
% This diagnostic distinguishes a real avoidance/recovery transient from a
% permanent reference/plant radial offset. It does not modify the Simulink
% model; it only runs the existing diagnostic logger and creates figures.

if nargin < 1 || isempty(stopTime)
    stopTime = 1500;
end

figDir = fullfile(pwd, "LaTeX", "GNSS", "figures", "gnss_sensor_validation");
if ~exist(figDir, "dir")
    mkdir(figDir);
end

diagMat = fullfile(figDir, sprintf("kalman_gnss_selection_diag_%ds.mat", round(stopTime)));
needsSimulation = true;
if isfile(diagMat)
    loaded = load(diagMat, "report");
    if isfield(loaded, "report") && isfield(loaded.report, "mpcU")
        report = loaded.report;
        needsSimulation = false;
    end
end

if needsSimulation
    fprintf("Running diagnostic with MPC control logging for %.0f s...\n", stopTime);
    report = diagnose_kalman_gnss_selection(stopTime);
end

t = report.t(:);
refPos = report.refPos;
selectedMinusRefEci = report.selectedMinusRef;
plantMinusRefEci = report.plantMinusRef;
mpcU = report.mpcU;

refVel = zeros(size(refPos));
for axisIdx = 1:3
    refVel(:, axisIdx) = gradient(refPos(:, axisIdx), t);
end

selectedErrRtn = eci_vector_to_rtn(selectedMinusRefEci, refPos, refVel);
plantErrRtn = eci_vector_to_rtn(plantMinusRefEci, refPos, refVel);
uRtn = eci_vector_to_rtn(mpcU, refPos, refVel);

selectedNorm = vecnorm(selectedErrRtn, 2, 2);
plantNorm = vecnorm(plantErrRtn, 2, 2);
uNorm = vecnorm(uRtn, 2, 2);

[peakErr, peakIdx] = max(selectedNorm);
peakTime = t(peakIdx);
tDebris = NaN;
if isfield(report, "tDebris")
    tDebris = report.tDebris;
end

beforeMask = t < max(0, peakTime - 120);
afterMask = t > min(t(end), peakTime + 450);
if nnz(beforeMask) < 5
    beforeMask = t < peakTime;
end
if nnz(afterMask) < 5
    afterMask = t > peakTime;
end

controlWin = abs(t - peakTime) <= 120;
debrisWin = false(size(t));
if ~isnan(tDebris)
    debrisWin = abs(t - tDebris) <= 120;
end

audit = struct();
audit.stopTime = stopTime;
audit.t = t;
audit.referenceSourceMode = "";
if isfield(report, "referenceSourceMode")
    audit.referenceSourceMode = report.referenceSourceMode;
end
audit.tDebris = tDebris;
audit.peakTime = peakTime;
audit.peakError = peakErr;
audit.selectedErrRtn = selectedErrRtn;
audit.plantErrRtn = plantErrRtn;
audit.uRtn = uRtn;
audit.selectedNorm = selectedNorm;
audit.plantNorm = plantNorm;
audit.uNorm = uNorm;
audit.radialAtStart = selectedErrRtn(1, 1);
audit.radialAtPeak = selectedErrRtn(peakIdx, 1);
audit.radialAtEnd = selectedErrRtn(end, 1);
audit.radialAbsMedianBeforePeak = median(abs(selectedErrRtn(beforeMask, 1)));
audit.radialAbsMedianAfterPeak = median(abs(selectedErrRtn(afterMask, 1)));
audit.maxAbsControlRtnNearPeak = max(abs(uRtn(controlWin, :)), [], 1);
if any(debrisWin)
    audit.maxAbsControlRtnNearDebris = max(abs(uRtn(debrisWin, :)), [], 1);
else
    audit.maxAbsControlRtnNearDebris = [NaN NaN NaN];
end

figPath = fullfile(figDir, sprintf("tracking_rtn_control_audit_%ds.png", round(stopTime)));
zoomPath = fullfile(figDir, sprintf("tracking_rtn_control_zoom_%ds.png", round(stopTime)));
matPath = fullfile(figDir, sprintf("tracking_rtn_control_audit_%ds.mat", round(stopTime)));

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1450 950]);
tiledlayout(4, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
hold on;
grid on;
plot(t, selectedNorm, "k", "LineWidth", 1.5, "DisplayName", "seleccionada-ref");
plot(t, plantNorm, "Color", [0.45 0.45 0.45], "LineWidth", 1.0, "DisplayName", "planta-ref");
mark_event_lines(peakTime, tDebris);
ylabel("Norma [m]");
title(sprintf("Error de seguimiento completo | pico %.1f m en t = %.1f s", peakErr, peakTime));
legend("Location", "best");

nexttile;
hold on;
grid on;
plot(t, selectedErrRtn(:,1), "Color", [0.85 0.33 0.10], "LineWidth", 1.2, "DisplayName", "\Delta R");
plot(t, selectedErrRtn(:,2), "Color", [0 0.45 0.75], "LineWidth", 1.2, "DisplayName", "\Delta T");
plot(t, selectedErrRtn(:,3), "Color", [0.47 0.67 0.19], "LineWidth", 1.2, "DisplayName", "\Delta N");
mark_event_lines(peakTime, tDebris);
ylabel("Error RTN [m]");
title("Componentes RTN del seguimiento seleccionado respecto a referencia");
legend("Location", "best");

nexttile;
hold on;
grid on;
plot(t, uRtn(:,1), "Color", [0.85 0.33 0.10], "LineWidth", 1.2, "DisplayName", "u_R");
plot(t, uRtn(:,2), "Color", [0 0.45 0.75], "LineWidth", 1.2, "DisplayName", "u_T");
plot(t, uRtn(:,3), "Color", [0.47 0.67 0.19], "LineWidth", 1.2, "DisplayName", "u_N");
mark_event_lines(peakTime, tDebris);
ylabel("u RTN [m/s^2]");
title("Control MPC proyectado al frame RTN de la referencia");
legend("Location", "best");

nexttile;
hold on;
grid on;
stairs(t, report.lambda, "Color", [0 0.35 0.7], "LineWidth", 1.3, "DisplayName", "\lambda");
plot(t, uNorm / max(max(uNorm), eps), "Color", [0.35 0.35 0.35], "LineWidth", 1.1, ...
    "DisplayName", "|u| normalizado");
mark_event_lines(peakTime, tDebris);
ylim([-0.1 1.1]);
xlabel("Tiempo [s]");
ylabel("\lambda / |u|");
title("Selector instrumental y actividad relativa de control");
legend("Location", "best");

exportgraphics(fig, figPath, "Resolution", 180);
close(fig);

zoomMask = t >= max(0, peakTime - 180) & t <= min(t(end), peakTime + 180);
fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1450 820]);
tiledlayout(3, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
hold on;
grid on;
plot(t(zoomMask), selectedNorm(zoomMask), "k", "LineWidth", 1.7, "DisplayName", "norma");
mark_event_lines(peakTime, tDebris);
ylabel("Norma [m]");
title("Zoom alrededor del pico de seguimiento");
legend("Location", "best");

nexttile;
hold on;
grid on;
plot(t(zoomMask), selectedErrRtn(zoomMask,1), "Color", [0.85 0.33 0.10], "LineWidth", 1.4, "DisplayName", "\Delta R");
plot(t(zoomMask), selectedErrRtn(zoomMask,2), "Color", [0 0.45 0.75], "LineWidth", 1.4, "DisplayName", "\Delta T");
plot(t(zoomMask), selectedErrRtn(zoomMask,3), "Color", [0.47 0.67 0.19], "LineWidth", 1.4, "DisplayName", "\Delta N");
mark_event_lines(peakTime, tDebris);
ylabel("Error RTN [m]");
legend("Location", "best");

nexttile;
hold on;
grid on;
plot(t(zoomMask), uRtn(zoomMask,1), "Color", [0.85 0.33 0.10], "LineWidth", 1.4, "DisplayName", "u_R");
plot(t(zoomMask), uRtn(zoomMask,2), "Color", [0 0.45 0.75], "LineWidth", 1.4, "DisplayName", "u_T");
plot(t(zoomMask), uRtn(zoomMask,3), "Color", [0.47 0.67 0.19], "LineWidth", 1.4, "DisplayName", "u_N");
mark_event_lines(peakTime, tDebris);
xlabel("Tiempo [s]");
ylabel("u RTN [m/s^2]");
legend("Location", "best");

exportgraphics(fig, zoomPath, "Resolution", 180);
close(fig);

audit.figPath = figPath;
audit.zoomPath = zoomPath;
audit.matPath = matPath;
save(matPath, "audit");

fprintf("\nTracking/control audit:\n");
fprintf("referenceSourceMode = %s\n", string(audit.referenceSourceMode));
fprintf("t_debris            = %.3f s\n", tDebris);
fprintf("peak selected-ref   = %.3f m at %.3f s\n", peakErr, peakTime);
fprintf("DeltaR start/peak/end = %.3f / %.3f / %.3f m\n", ...
    audit.radialAtStart, audit.radialAtPeak, audit.radialAtEnd);
fprintf("median |DeltaR| before peak = %.3f m\n", audit.radialAbsMedianBeforePeak);
fprintf("median |DeltaR| after peak  = %.3f m\n", audit.radialAbsMedianAfterPeak);
fprintf("max |u_R,u_T,u_N| near peak = [%.4g %.4g %.4g] m/s2\n", audit.maxAbsControlRtnNearPeak);
fprintf("max |u_R,u_T,u_N| near debris = [%.4g %.4g %.4g] m/s2\n", audit.maxAbsControlRtnNearDebris);
fprintf("Saved audit figure: %s\n", figPath);
fprintf("Saved zoom figure:  %s\n", zoomPath);

end

function mark_event_lines(peakTime, tDebris)
xline(peakTime, "--", "Color", [0.55 0.55 0.55], "LineWidth", 1.0, ...
    "DisplayName", "pico");
if ~isnan(tDebris)
    xline(tDebris, ":", "Color", [0.6 0.1 0.7], "LineWidth", 1.2, ...
        "DisplayName", "debris");
end
end

function errRtn = eci_vector_to_rtn(vecEci, refPos, refVel)
n = size(vecEci, 1);
errRtn = zeros(n, 3);
for k = 1:n
    r = refPos(k, :).';
    v = refVel(k, :).';
    rNorm = norm(r);
    h = cross(r, v);
    hNorm = norm(h);
    if rNorm < eps || hNorm < eps
        C = eye(3);
    else
        rHat = r / rNorm;
        nHat = h / hNorm;
        tHat = cross(nHat, rHat);
        C = [rHat, tHat, nHat];
    end
    errRtn(k, :) = (C.' * vecEci(k, :).').';
end
end

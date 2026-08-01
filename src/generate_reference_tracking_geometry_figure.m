function out = generate_reference_tracking_geometry_figure(stopTime, windowHalfWidth)
%GENERATE_REFERENCE_TRACKING_GEOMETRY_FIGURE Build an illustrative tracking plot.
%
% The figure compares the currently selected navigation signal against the
% reference trajectory. Error components are expressed in the local RTN
% frame of the reference trajectory:
%   R = radial, T = along-track, N = cross-track.

if nargin < 1 || isempty(stopTime)
    stopTime = 1500;
end
if nargin < 2 || isempty(windowHalfWidth)
    windowHalfWidth = 80;
end

figDir = fullfile(pwd, "LaTeX", "GNSS", "figures", "gnss_sensor_validation");
if ~exist(figDir, "dir")
    mkdir(figDir);
end

diagMat = fullfile(figDir, sprintf("kalman_gnss_selection_diag_%ds.mat", round(stopTime)));
needsSimulation = true;
if isfile(diagMat)
    loaded = load(diagMat, "report");
    if isfield(loaded, "report") && isfield(loaded.report, "selectedMinusRef")
        report = loaded.report;
        needsSimulation = false;
    end
end

if needsSimulation
    fprintf("Diagnostic vectors not found for %.0f s. Running Simulink diagnostic...\n", stopTime);
    diagnosticLog = evalc("report = diagnose_kalman_gnss_selection(stopTime);");
    logPath = fullfile(figDir, sprintf("reference_tracking_geometry_simlog_%ds.txt", round(stopTime)));
    fid = fopen(logPath, "w");
    fprintf(fid, "%s", diagnosticLog);
    fclose(fid);
    fprintf("Saved captured Simulink log: %s\n", logPath);
end

t = report.t(:);
refPos = report.refPos;
if isfield(report, "selectedPos")
    selectedPos = report.selectedPos;
else
    selectedPos = refPos + report.selectedMinusRef;
end
selectedMinusRefEci = report.selectedMinusRef;

% Reconstruct reference velocity from finite differences. The RTN basis is
% only used for visualization, so this is sufficient and avoids depending on
% extra logged signals.
refVel = zeros(size(refPos));
for dimIdx = 1:3
    refVel(:, dimIdx) = gradient(refPos(:, dimIdx), t);
end
selectedMinusRefRtn = eci_error_to_rtn(selectedMinusRefEci, refPos, refVel);
trackingNorm = vecnorm(selectedMinusRefRtn, 2, 2);

[peakErr, peakIdx] = max(trackingNorm);
peakTime = t(peakIdx);
win = (t >= peakTime - windowHalfWidth) & (t <= peakTime + windowHalfWidth);
if nnz(win) < 5
    error("Not enough samples in plotting window around the peak.");
end

tWin = t(win);
errWin = trackingNorm(win);
compWin = selectedMinusRefRtn(win, :);
winIdx = find(win);
peakWinIdx = find(winIdx == peakIdx, 1);
if isempty(peakWinIdx)
    [~, peakWinIdx] = min(abs(tWin - peakTime));
end

% 3-D trajectory view: both curves are plotted in the same position frame.
% We subtract only the first reference sample in the window to keep the
% numbers readable. We do not subtract reference(t) at every instant here.
plotOrigin = refPos(winIdx(1), :);
refLocal3d = refPos(win, :) - plotOrigin;
selectedLocal3d = selectedPos(win, :) - plotOrigin;

dt = median(diff(tWin));
smoothWindow = max(3, 2 * floor((18 / dt) / 2) + 1);
errSmooth = movmean(errWin, smoothWindow);
meanErr = mean(errWin);
rmsErr = sqrt(mean(errWin.^2));

figPath = fullfile(figDir, sprintf("reference_tracking_geometry_3d_consistent_%ds.png", round(stopTime)));
legacyFigPath = fullfile(figDir, sprintf("reference_tracking_geometry_current_%ds.png", round(stopTime)));
correctedFigPath = fullfile(figDir, sprintf("reference_tracking_geometry_rtn_planes_%ds.png", round(stopTime)));
error3dFigPath = fullfile(figDir, sprintf("reference_tracking_geometry_rtn_error_3d_%ds.png", round(stopTime)));
matPath = fullfile(figDir, sprintf("reference_tracking_geometry_current_%ds.mat", round(stopTime)));

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1500 900]);
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile([2 1]);
hold on;
grid on;
box on;
plot3(refLocal3d(:,1), refLocal3d(:,2), refLocal3d(:,3), "--", ...
    "Color", [0 0.45 0.75], "LineWidth", 2.0, "DisplayName", "Referencia");
plot3(selectedLocal3d(:,1), selectedLocal3d(:,2), selectedLocal3d(:,3), ...
    "Color", [0.85 0.33 0.10], "LineWidth", 2.0, "DisplayName", "Señal seleccionada");
plot3([refLocal3d(peakWinIdx,1), selectedLocal3d(peakWinIdx,1)], ...
    [refLocal3d(peakWinIdx,2), selectedLocal3d(peakWinIdx,2)], ...
    [refLocal3d(peakWinIdx,3), selectedLocal3d(peakWinIdx,3)], ...
    ":", "Color", [0.15 0.15 0.15], "LineWidth", 1.6, "DisplayName", "Error en pico");
scatter3(selectedLocal3d(peakWinIdx,1), selectedLocal3d(peakWinIdx,2), selectedLocal3d(peakWinIdx,3), ...
    85, "filled", "MarkerFaceColor", [1 0.92 0], "MarkerEdgeColor", "k", ...
    "DisplayName", "Pico de error");
xlabel("\Delta x respecto al origen local [m]");
ylabel("\Delta y respecto al origen local [m]");
zlabel("\Delta z respecto al origen local [m]");
title(sprintf("Geometría 3D coherente | pico = %.1f m en t = %.1f s", peakErr, peakTime));
legend("Location", "best");
view(38, 22);
axis tight;
axis equal;
axis vis3d;

nexttile;
hold on;
grid on;
plot(tWin, errWin, "k", "LineWidth", 1.8, "DisplayName", "Error instantáneo");
plot(tWin, errSmooth, "Color", [0 0.45 0.75], "LineWidth", 2.0, "DisplayName", "Tendencia suavizada");
xline(peakTime, "--", "Color", [0.5 0.5 0.5], "LineWidth", 1.2, "DisplayName", "Tiempo de pico");
yline(meanErr, "--", "Color", [0.85 0.33 0.10], "LineWidth", 1.2, "DisplayName", "Media ventana");
xlabel("t [s]");
ylabel("Norma del error [m]");
title(sprintf("Norma del error de seguimiento | media = %.1f m | RMS = %.1f m", meanErr, rmsErr));
legend("Location", "best");

nexttile;
hold on;
grid on;
plot(tWin, compWin(:,1), "Color", [0.85 0.33 0.10], "LineWidth", 1.7, "DisplayName", "\Delta R");
plot(tWin, compWin(:,2), "Color", [0 0.45 0.75], "LineWidth", 1.7, "DisplayName", "\Delta T");
plot(tWin, compWin(:,3), "Color", [0.47 0.67 0.19], "LineWidth", 1.7, "DisplayName", "\Delta N");
yline(0, "-", "Color", [0.3 0.3 0.3], "HandleVisibility", "off");
xline(peakTime, "--", "Color", [0.5 0.5 0.5], "LineWidth", 1.2, "DisplayName", "Tiempo de pico");
xlabel("t [s]");
ylabel("Componente del error [m]");
title("Componentes RTN del error de seguimiento");
legend("Location", "best");

exportgraphics(fig, figPath, "Resolution", 180);
exportgraphics(fig, legacyFigPath, "Resolution", 180);
close(fig);

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1600 900]);
tiledlayout(2, 3, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot_error_plane(compWin(:,2), compWin(:,1), compWin(tWin == peakTime,2), compWin(tWin == peakTime,1), ...
    "\Delta T along-track [m]", "\Delta R radial [m]", "Plano R-T");

nexttile;
plot_error_plane(compWin(:,3), compWin(:,1), compWin(tWin == peakTime,3), compWin(tWin == peakTime,1), ...
    "\Delta N cross-track [m]", "\Delta R radial [m]", "Plano R-N");

nexttile;
plot_error_plane(compWin(:,2), compWin(:,3), compWin(tWin == peakTime,2), compWin(tWin == peakTime,3), ...
    "\Delta T along-track [m]", "\Delta N cross-track [m]", "Plano T-N");

nexttile;
hold on;
grid on;
plot(tWin, errWin, "k", "LineWidth", 1.8, "DisplayName", "Error instantáneo");
plot(tWin, errSmooth, "Color", [0 0.45 0.75], "LineWidth", 2.0, "DisplayName", "Tendencia suavizada");
xline(peakTime, "--", "Color", [0.5 0.5 0.5], "LineWidth", 1.2, "DisplayName", "Tiempo de pico");
yline(meanErr, "--", "Color", [0.85 0.33 0.10], "LineWidth", 1.2, "DisplayName", "Media ventana");
xlabel("t [s]");
ylabel("Norma del error [m]");
title(sprintf("Norma | pico %.1f m | media %.1f m | RMS %.1f m", peakErr, meanErr, rmsErr));
legend("Location", "best");

nexttile([1 2]);
hold on;
grid on;
plot(tWin, compWin(:,1), "Color", [0.85 0.33 0.10], "LineWidth", 1.7, "DisplayName", "\Delta R");
plot(tWin, compWin(:,2), "Color", [0 0.45 0.75], "LineWidth", 1.7, "DisplayName", "\Delta T");
plot(tWin, compWin(:,3), "Color", [0.47 0.67 0.19], "LineWidth", 1.7, "DisplayName", "\Delta N");
yline(0, "-", "Color", [0.3 0.3 0.3], "HandleVisibility", "off");
xline(peakTime, "--", "Color", [0.5 0.5 0.5], "LineWidth", 1.2, "DisplayName", "Tiempo de pico");
xlabel("t [s]");
ylabel("Componente del error [m]");
title("Componentes RTN del error de seguimiento");
legend("Location", "best");

exportgraphics(fig, correctedFigPath, "Resolution", 180);
close(fig);

fig = figure("Visible", "off", "Color", "w", "Position", [80 80 1200 900]);
hold on;
grid on;
box on;
plot3(compWin(:,2), compWin(:,1), compWin(:,3), ...
    "Color", [0.85 0.33 0.10], "LineWidth", 2.1, "DisplayName", "Error seleccionado");
scatter3(0, 0, 0, 80, "filled", "MarkerFaceColor", [0 0.45 0.75], ...
    "MarkerEdgeColor", "k", "DisplayName", "Referencia");
plot3([0, compWin(peakWinIdx,2)], [0, compWin(peakWinIdx,1)], [0, compWin(peakWinIdx,3)], ...
    ":", "Color", [0.15 0.15 0.15], "LineWidth", 1.8, "DisplayName", "Error en pico");
scatter3(compWin(peakWinIdx,2), compWin(peakWinIdx,1), compWin(peakWinIdx,3), ...
    95, "filled", "MarkerFaceColor", [1 0.92 0], "MarkerEdgeColor", "k", ...
    "DisplayName", "Pico de error");
axisLimit = max(abs(compWin), [], "all");
plot3([-axisLimit axisLimit], [0 0], [0 0], "-", "Color", [0.75 0.75 0.75], "HandleVisibility", "off");
plot3([0 0], [-axisLimit axisLimit], [0 0], "-", "Color", [0.75 0.75 0.75], "HandleVisibility", "off");
plot3([0 0], [0 0], [-axisLimit axisLimit], "-", "Color", [0.75 0.75 0.75], "HandleVisibility", "off");
xlabel("\Delta T along-track [m]");
ylabel("\Delta R radial [m]");
zlabel("\Delta N cross-track [m]");
title(sprintf("Geometría 3D del error RTN | pico = %.1f m en t = %.1f s", peakErr, peakTime));
legend("Location", "best");
view(42, 24);
axis equal;
axis tight;
axis vis3d;
exportgraphics(fig, error3dFigPath, "Resolution", 180);
close(fig);

out = struct();
out.stopTime = stopTime;
out.windowHalfWidth = windowHalfWidth;
out.peakTime = peakTime;
out.peakError = peakErr;
out.windowMean = meanErr;
out.windowRms = rmsErr;
out.figPath = figPath;
out.legacyFigPath = legacyFigPath;
out.correctedFigPath = correctedFigPath;
out.error3dFigPath = error3dFigPath;
out.matPath = matPath;
out.tWindow = tWin;
out.errorWindow = errWin;
out.errorComponentsRtnWindow = compWin;
save(matPath, "out");

fprintf("Saved reference-tracking geometry figure: %s\n", figPath);
fprintf("Saved corrected RTN plane figure: %s\n", correctedFigPath);
fprintf("Saved RTN 3-D error figure: %s\n", error3dFigPath);
fprintf("Peak %.3f m at t = %.3f s | window mean %.3f m | window RMS %.3f m\n", ...
    peakErr, peakTime, meanErr, rmsErr);

end

function plot_error_plane(x, y, xPeak, yPeak, xLabelText, yLabelText, titleText)
hold on;
grid on;
box on;
plot(x, y, "Color", [0.85 0.33 0.10], "LineWidth", 1.8, "DisplayName", "Error relativo");
scatter(0, 0, 60, "filled", "MarkerFaceColor", [0 0.45 0.75], ...
    "MarkerEdgeColor", "k", "DisplayName", "Referencia local");
scatter(xPeak, yPeak, 80, "filled", "MarkerFaceColor", [1 0.92 0], ...
    "MarkerEdgeColor", "k", "DisplayName", "Pico");
xline(0, "-", "Color", [0.65 0.65 0.65], "HandleVisibility", "off");
yline(0, "-", "Color", [0.65 0.65 0.65], "HandleVisibility", "off");
xlabel(xLabelText);
ylabel(yLabelText);
title(titleText);
axis equal;
legend("Location", "best");
end

function errRtn = eci_error_to_rtn(errEci, refPos, refVel)
n = size(errEci, 1);
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
    errRtn(k, :) = (C.' * errEci(k, :).').';
end
end

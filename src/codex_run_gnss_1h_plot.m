clearvars;
close all;
clc;

modelName = "MPCcontrolledSpacecraft_plant_gnss_no_predictor";
codexStopTime = 3600;

bdclose all;
load_system(modelName);
set_param(modelName, "StopTime", "3600");

batchText = evalc("Script_Concurso_Spacecraft_batch;");

if exist("Np", "var") && exist("h", "var")
    requiredRefEnd = codexStopTime + Np*h;
else
    requiredRefEnd = codexStopTime;
end

if exist("t_ref", "var")
    fprintf("t_ref(end) = %.3f s | requerido aprox >= %.3f s\n", ...
        t_ref(end), requiredRefEnd);

    if t_ref(end) < requiredRefEnd
        warning("La referencia es demasiado corta. Puede aparecer una explosion numerica al final.");
    end
end

simText = evalc("simOut = sim(modelName);");
consoleText = [batchText newline simText];

tokensErr = regexp( ...
    consoleText, ...
    "t=([0-9]+(?:\.[0-9]+)?)\s*\|\s*exit=.*?\|\s*viol=.*?\|\s*maxU=.*?\|\s*err=([0-9.eE+-]+)\s*m", ...
    "tokens");

if isempty(tokensErr)
    error("No se han encontrado lineas tipo 'err=... m' en la salida de MATLAB.");
end

tErr = zeros(numel(tokensErr), 1);
errPos = zeros(numel(tokensErr), 1);

for k = 1:numel(tokensErr)
    tErr(k) = str2double(tokensErr{k}{1});
    errPos(k) = str2double(tokensErr{k}{2});
end

tokensXrel = regexp( ...
    consoleText, ...
    "t=([0-9]+(?:\.[0-9]+)?)\s*\|\s*norm x_rel\s*=\s*([0-9.eE+-]+)\s*\|\s*norm Y0 pos first\s*=\s*([0-9.eE+-]+)", ...
    "tokens");

tXrel = [];
xRelNorm = [];

if ~isempty(tokensXrel)
    tXrel = zeros(numel(tokensXrel), 1);
    xRelNorm = zeros(numel(tokensXrel), 1);

    for k = 1:numel(tokensXrel)
        tXrel(k) = str2double(tokensXrel{k}{1});
        xRelNorm(k) = str2double(tokensXrel{k}{2});
    end
end

figDir = fullfile(pwd, "LaTeX", "GNSS", "figures", "gnss_sensor_validation");

if ~exist(figDir, "dir")
    mkdir(figDir);
end

figPath = fullfile(figDir, "gnss_reference_position_error_1h_thresholds.png");
resultPath = fullfile(figDir, "gnss_reference_position_error_1h_results.mat");
consolePath = fullfile(figDir, "gnss_reference_position_error_1h_console.txt");

figure("Color", "w", "Position", [100 100 1300 600]);
hold on;
grid on;

if ~isempty(tXrel)
    plot(tXrel, xRelNorm, "-", ...
        "Color", [0.55 0.75 0.95], ...
        "LineWidth", 1.0, ...
        "DisplayName", "norm x\_rel cada 3 s");
end

plot(tErr, errPos, "-o", ...
    "Color", [0.0 0.32 0.62], ...
    "LineWidth", 2.2, ...
    "MarkerSize", 4, ...
    "DisplayName", "err MPC cada 30 s");

yline(50, "--", "50 m", ...
    "Color", [0.95 0.45 0.05], ...
    "LineWidth", 1.8, ...
    "LabelHorizontalAlignment", "right");

yline(20, "--", "20 m", ...
    "Color", [0.1 0.6 0.15], ...
    "LineWidth", 1.8, ...
    "LabelHorizontalAlignment", "right");

[maxErr, idxMax] = max(errPos);

scatter(tErr(idxMax), maxErr, 60, "r", "filled", ...
    "DisplayName", sprintf("Pico %.1f m @ %.0f s", maxErr, tErr(idxMax)));

xlabel("Tiempo [s]");
ylabel("Norma del error de posicion [m]");
title("Error de posicion GNSS frente a trayectoria de referencia - 1 hora");
legend("Location", "best");

xlim([0 3600]);
ylim([0 max(maxErr*1.12, 60)]);

exportgraphics(gcf, figPath, "Resolution", 200);

save(resultPath, "tErr", "errPos", "tXrel", "xRelNorm", "consoleText");

fid = fopen(consolePath, "w");
if fid >= 0
    fwrite(fid, consoleText);
    fclose(fid);
end

fprintf("Figura guardada en:\n%s\n", figPath);
fprintf("Resultados guardados en:\n%s\n", resultPath);
fprintf("Salida de consola guardada en:\n%s\n", consolePath);
fprintf("Resumen: t_final=%.1f s | err_final=%.3f m | err_max=%.3f m en t=%.1f s\n", ...
    tErr(end), errPos(end), maxErr, tErr(idxMax));

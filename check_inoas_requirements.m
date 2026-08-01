function check_inoas_requirements()
%CHECK_INOAS_REQUIREMENTS Warn about missing MATLAB products and data files.

installedToolboxes = ver;
installedProducts = string({installedToolboxes.Name});
requiredProducts = [
    "Simulink"
    "Aerospace Blockset"
    "Aerospace Toolbox"
    "Optimization Toolbox"
];

missingProducts = requiredProducts(~ismember(requiredProducts, installedProducts));
if ~isempty(missingProducts)
    warning('INOAS:MissingProducts', ...
        'Missing MATLAB products: %s', char(strjoin(missingProducts, ", ")));
end

ephemerisFiles = [
    "ephMoon405.mat"
    "ephEarthMoonBarycenter405.mat"
    "ephSun405.mat"
];

missingEphemerisFiles = strings(0, 1);
for k = 1:numel(ephemerisFiles)
    if isempty(which(char(ephemerisFiles(k))))
        missingEphemerisFiles(end + 1, 1) = ephemerisFiles(k); %#ok<AGROW>
    end
end

if ~isempty(missingEphemerisFiles)
    warning('INOAS:MissingEphemerisData', ...
        ['Missing Aerospace Toolbox ephemeris data files: %s\n' ...
        'Install the MATLAB add-on ''Ephemeris Data for Aerospace Toolbox'' ' ...
        'before running the Simulink simulation.'], ...
        char(strjoin(missingEphemerisFiles, ", ")));
end
end

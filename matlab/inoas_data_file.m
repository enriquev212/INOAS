function filePath = inoas_data_file(filename)
%INOAS_DATA_FILE Return the absolute path to a file stored in data/.

if nargin < 1 || strlength(string(filename)) == 0
    error("inoas_data_file:missingFilename", "A data filename is required.");
end

filename = string(filename);

if isfile(filename)
    filePath = char(filename);
    return;
end

functionDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(functionDir);
candidate = fullfile(repoRoot, "data", filename);

if isfile(candidate)
    filePath = char(candidate);
    return;
end

candidate = fullfile(pwd, "data", filename);

if isfile(candidate)
    filePath = char(candidate);
    return;
end

error("inoas_data_file:notFound", "Data file not found: %s", filename);
end

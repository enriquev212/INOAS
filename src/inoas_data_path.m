function filePath = inoas_data_path(filename)
%INOAS_DATA_PATH Return the preferred path for a file stored in data/.

if nargin < 1 || strlength(string(filename)) == 0
    error("inoas_data_path:missingFilename", "A data filename is required.");
end

srcDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(srcDir);
dataDir = fullfile(repoRoot, "data");

if ~isfolder(dataDir)
    mkdir(dataDir);
end

filePath = char(fullfile(dataDir, string(filename)));
end

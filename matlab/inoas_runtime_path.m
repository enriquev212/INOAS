function filename = inoas_runtime_path(name)
% Generated trajectories are caches, not versioned sensor inputs.
root = fileparts(fileparts(mfilename('fullpath')));
directory = fullfile(root, 'results', 'cache');
if ~isfolder(directory), mkdir(directory); end
filename = fullfile(directory, name);
end

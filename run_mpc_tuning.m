function [best, results] = run_mpc_tuning(options)
%RUN_MPC_TUNING Convenience entry point for automated INOAS MPC tuning.
%
% From the repository root:
%   [best, results] = run_mpc_tuning;
%
% Optional reduced campaign:
%   opt = struct('NumGroupCandidates',10,'NumLocalCandidates',16, ...
%                'NumFullValidation',3);
%   [best, results] = run_mpc_tuning(opt);

if nargin < 1
    options = struct();
end

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
addpath(genpath(fullfile(projectRoot, 'matlab')));
addpath(fullfile(projectRoot, 'models'));

[best, results] = tune_mpc_deltaV(options);
end

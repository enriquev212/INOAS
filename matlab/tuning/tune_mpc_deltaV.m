function [best, results] = tune_mpc_deltaV(options)
%TUNE_MPC_DELTAV Tune Q_step, R_step and S_step for minimum total delta-V.
%
% The search is deliberately derivative-free because the outer objective is
% a complete Simulink run containing constrained MPC, saturations and mode
% logic. The inner MPC remains unchanged and still uses fmincon.
%
% Usage from the INOAS repository root:
%   [best, results] = tune_mpc_deltaV;
%
% Faster exploratory campaign:
%   opt = struct('NumGroupCandidates', 12, 'NumLocalCandidates', 16, ...
%                'NumFullValidation', 4);
%   [best, results] = tune_mpc_deltaV(opt);
%
% The final winner is selected ONLY among functional candidates:
%   1) finite simulation signals,
%   2) non-negative covariance-aware debris margin,
%   3) no actuator-box violation (within numerical tolerance),
%   4) final and post-encounter tracking no worse than the baseline-derived
%      limits below.

if nargin < 1 || isempty(options)
    options = struct();
end

defaults = struct( ...
    'NumGroupCandidates', 18, ...
    'NumLocalCandidates', 32, ...
    'NumFullValidation', 6, ...
    'CoarseStopTime', 2100, ...
    'FullStopTime', 4000, ...
    'RandomSeed', 240526, ...
    'TrackingRelativeTolerance', 0.25, ...
    'TrackingAbsoluteTolerance', 5, ...
    'SafetyBuffer', 0, ...
    'WeightMultiplierMin', 0.1, ...
    'WeightMultiplierMax', 10);

userFields = fieldnames(options);
for k = 1:numel(userFields)
    if ~isfield(defaults, userFields{k})
        error('Unknown tuning option: %s', userFields{k});
    end
    defaults.(userFields{k}) = options.(userFields{k});
end
options = defaults;

validateattributes(options.NumGroupCandidates, {'numeric'}, {'scalar','integer','nonnegative'});
validateattributes(options.NumLocalCandidates, {'numeric'}, {'scalar','integer','nonnegative'});
validateattributes(options.NumFullValidation, {'numeric'}, {'scalar','integer','positive'});
validateattributes(options.CoarseStopTime, {'numeric'}, {'scalar','positive'});
validateattributes(options.FullStopTime, {'numeric'}, {'scalar','positive'});
validateattributes(options.TrackingRelativeTolerance, {'numeric'}, {'scalar','nonnegative'});
validateattributes(options.TrackingAbsoluteTolerance, {'numeric'}, {'scalar','nonnegative'});
validateattributes(options.WeightMultiplierMin, {'numeric'}, {'scalar','positive'});
validateattributes(options.WeightMultiplierMax, {'numeric'}, {'scalar','positive'});

if options.WeightMultiplierMax <= options.WeightMultiplierMin
    error('WeightMultiplierMax must be greater than WeightMultiplierMin.');
end

thisFile = mfilename('fullpath');
tuningDir = fileparts(thisFile);
matlabDir = fileparts(tuningDir);
projectRoot = fileparts(matlabDir);
modelName = "inoas_model";
modelPath = fullfile(projectRoot, 'models', modelName + ".slx");
initPath = fullfile(projectRoot, 'initialize_inoas_simulation.m');
resultsDir = fullfile(projectRoot, 'tuning_results');
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

oldDir = pwd;
cleanupDir = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(projectRoot);
addpath(projectRoot);
addpath(genpath(matlabDir));
addpath(fullfile(projectRoot, 'models'));

fprintf('\n============================================================\n');
fprintf('INOAS outer-loop MPC tuning: minimum total Delta-V\n');
fprintf('Only Q_step, R_step and S_step are modified.\n');
fprintf('============================================================\n\n');

% Initialize the exact scenario defined by initialize_inoas_simulation.m.
assignin('base', 'modelName', modelName);
assignin('base', 'simulationStopTime', options.FullStopTime);
evalin('base', sprintf('run(''%s'')', strrep(initPath, '''', '''''')));
load_system(modelPath);

baseWeights = struct();
baseWeights.Q_step = evalin('base', 'Q_step');
baseWeights.R_step = evalin('base', 'R_step');
baseWeights.S_step = evalin('base', 'S_step');

fprintf('Scenario taken from initialize_inoas_simulation.m:\n');
fprintf('  Np = %d, h = %.3f s, t_debris = %.3f s, dsafe0 = %.3f m\n', ...
    evalin('base','Np'), evalin('base','h'), evalin('base','t_debris'), evalin('base','dsafe0'));
fprintf('Baseline Q_step = [%s]\n', num2str(baseWeights.Q_step, ' %.6g'));
fprintf('Baseline R_step = [%s]\n', num2str(baseWeights.R_step, ' %.6g'));
fprintf('Baseline S_step = [%s]\n\n', num2str(baseWeights.S_step, ' %.6g'));

% Run baseline on both horizons. The baseline itself defines the tracking
% performance that candidates are not allowed to degrade materially.
fprintf('Running coarse baseline...\n');
baseCoarse = run_mpc_tuning_case(modelName, baseWeights, options.CoarseStopTime);
assert(baseCoarse.ok, 'Baseline coarse simulation failed:\n%s', baseCoarse.message);
coarseLimits = makeFunctionalLimits(baseCoarse.metrics, options);
baseCoarse.functional = isFunctional(baseCoarse.metrics, coarseLimits);
baseCoarse.stage = "baseline-coarse";
printCase('BASE-COARSE', baseCoarse, baseCoarse.functional);

fprintf('Running full baseline...\n');
baseFull = run_mpc_tuning_case(modelName, baseWeights, options.FullStopTime);
assert(baseFull.ok, 'Baseline full simulation failed:\n%s', baseFull.message);
fullLimits = makeFunctionalLimits(baseFull.metrics, options);
baseFull.functional = isFunctional(baseFull.metrics, fullLimits);
baseFull.stage = "baseline-full";
printCase('BASE-FULL', baseFull, baseFull.functional);

if ~baseCoarse.functional || ~baseFull.functional
    error(['The current baseline does not satisfy the automatic definition of ', ...
           'functional. Fix/adjust the baseline or the thresholds before tuning.']);
end

rng(options.RandomSeed, 'twister');

% ---------- Stage 1: group scaling ---------------------------------------
% Search physically meaningful relative tradeoffs before opening all 12
% degrees of freedom. Log scaling is used because weight magnitudes span many
% orders of magnitude.
structuredLogs = [ ...
     0.00,  0.00,  0.00;
    -0.30,  0.30,  0.00;
    -0.30,  0.60,  0.30;
    -0.50,  0.70,  0.50;
     0.00,  0.30,  0.30;
     0.20,  0.50,  0.50;
    -0.20,  0.80,  0.20;
    -0.20,  0.20,  0.80];

nRandomGroup = max(0, options.NumGroupCandidates - size(structuredLogs, 1));
randomGroup = [ ...
    -0.70 + 1.10*rand(nRandomGroup,1), ... % Q: 0.20x to 2.51x
    -0.30 + 1.30*rand(nRandomGroup,1), ... % R: 0.50x to 10x
    -0.30 + 1.30*rand(nRandomGroup,1)];    % S: 0.50x to 10x

groupLogs = [structuredLogs; randomGroup];
if options.NumGroupCandidates < size(groupLogs,1)
    groupLogs = groupLogs(1:options.NumGroupCandidates,:);
end

coarseRuns = repmat(emptyRunStruct(), 0, 1);
for i = 1:size(groupLogs,1)
    mult = 10.^groupLogs(i,:);
    w = struct( ...
        'Q_step', baseWeights.Q_step * mult(1), ...
        'R_step', baseWeights.R_step * mult(2), ...
        'S_step', baseWeights.S_step * mult(3));
    w = clampWeights(w, baseWeights, options);
    run = run_mpc_tuning_case(modelName, w, options.CoarseStopTime);
    run.functional = run.ok && isFunctional(run.metrics, coarseLimits);
    run.stage = "group";
    coarseRuns(end+1,1) = run; %#ok<AGROW>
    printCase(sprintf('GROUP %02d/%02d', i, size(groupLogs,1)), run, run.functional);
    saveCheckpoint(resultsDir, baseWeights, baseCoarse, baseFull, coarseRuns, [], options);
end

% Pick the best functional coarse candidate so far. Baseline is always an
% eligible seed, ensuring the local stage cannot start from a failed point.
seedRun = baseCoarse;
seedRun.stage = "baseline";
functionalGroup = coarseRuns([coarseRuns.functional]);
if ~isempty(functionalGroup)
    [~, idx] = min(arrayfun(@(x) x.metrics.deltaV, functionalGroup));
    if functionalGroup(idx).metrics.deltaV < seedRun.metrics.deltaV
        seedRun = functionalGroup(idx);
    end
end

% ---------- Stage 2: independent 12-weight refinement -------------------
% Each local candidate perturbs every Q/R/S component independently around
% the best group-scaled candidate, allowing axis-specific tuning.
localRuns = repmat(emptyRunStruct(), 0, 1);
for i = 1:options.NumLocalCandidates
    % First 24 candidates deliberately probe every Q/R/S component in
    % both directions; later candidates explore coupled interactions.
    if i <= 24
        p = zeros(1,12);
        direction = 1;
        if mod(i,2) == 0
            direction = -1;
        end
        component = ceil(i/2);
        p(component) = direction * 0.30;
    else
        p = 0.28 * randn(1,12);
        p = max(min(p, 0.55), -0.55);
    end

    vecSeed = [seedRun.Q_step, seedRun.R_step, seedRun.S_step];
    vec = vecSeed .* (10.^p);
    w = struct('Q_step', vec(1:6), 'R_step', vec(7:9), 'S_step', vec(10:12));
    w = clampWeights(w, baseWeights, options);

    run = run_mpc_tuning_case(modelName, w, options.CoarseStopTime);
    run.functional = run.ok && isFunctional(run.metrics, coarseLimits);
    run.stage = "local";
    localRuns(end+1,1) = run; %#ok<AGROW>
    printCase(sprintf('LOCAL %02d/%02d', i, options.NumLocalCandidates), run, run.functional);

    % Opportunistic update: subsequent random candidates are centered on the
    % best functional point discovered so far.
    if run.functional && run.metrics.deltaV < seedRun.metrics.deltaV
        seedRun = run;
    end
    saveCheckpoint(resultsDir, baseWeights, baseCoarse, baseFull, coarseRuns, localRuns, options);
end

% ---------- Stage 3: full 4000 s validation of the best coarse points ----
allCoarse = [coarseRuns; localRuns];
functionalIdx = find([allCoarse.functional]);
if isempty(functionalIdx)
    candidates = seedRun;
else
    dV = arrayfun(@(x) x.metrics.deltaV, allCoarse(functionalIdx));
    [~, order] = sort(dV, 'ascend');
    nKeep = min(options.NumFullValidation, numel(order));
    candidates = allCoarse(functionalIdx(order(1:nKeep)));
end

% Always include the baseline full result separately for comparison.
fullRuns = repmat(emptyRunStruct(), 0, 1);
for i = 1:numel(candidates)
    w = struct('Q_step', candidates(i).Q_step, ...
               'R_step', candidates(i).R_step, ...
               'S_step', candidates(i).S_step);
    run = run_mpc_tuning_case(modelName, w, options.FullStopTime);
    run.functional = run.ok && isFunctional(run.metrics, fullLimits);
    run.stage = "full";
    fullRuns(end+1,1) = run; %#ok<AGROW>
    printCase(sprintf('FULL %02d/%02d', i, numel(candidates)), run, run.functional);
    saveCheckpoint(resultsDir, baseWeights, baseCoarse, baseFull, coarseRuns, localRuns, options, fullRuns);
end

% Winner is the minimum full-horizon Delta-V among functional candidates,
% with the original baseline included in the comparison.
best = baseFull;
best.stage = "baseline-full";
for i = 1:numel(fullRuns)
    if fullRuns(i).functional && fullRuns(i).metrics.deltaV < best.metrics.deltaV
        best = fullRuns(i);
    end
end
best.functional = true;

results = struct();
results.options = options;
results.baseWeights = baseWeights;
results.baselineCoarse = baseCoarse;
results.baselineFull = baseFull;
results.coarseLimits = coarseLimits;
results.fullLimits = fullLimits;
results.groupRuns = coarseRuns;
results.localRuns = localRuns;
results.fullRuns = fullRuns;
results.best = best;
results.generatedAt = datetime('now');

stamp = datestr(now, 'yyyymmdd_HHMMSS');
matFile = fullfile(resultsDir, ['mpc_tuning_' stamp '.mat']);
save(matFile, 'results', 'best');
summaryFile = fullfile(resultsDir, ['mpc_tuning_' stamp '.csv']);
writetable(buildSummaryTable(baseFull, coarseRuns, localRuns, fullRuns), summaryFile);
bestFile = fullfile(resultsDir, 'best_mpc_weights.m');
writeBestWeights(bestFile, best, baseFull);

fprintf('\n============================================================\n');
fprintf('BEST FUNCTIONAL FULL-HORIZON TUNING\n');
fprintf('============================================================\n');
fprintf('Q_step = [%s];\n', num2str(best.Q_step, ' %.12g'));
fprintf('R_step = [%s];\n', num2str(best.R_step, ' %.12g'));
fprintf('S_step = [%s];\n', num2str(best.S_step, ' %.12g'));
fprintf('Delta-V baseline = %.6f m/s\n', baseFull.metrics.deltaV);
fprintf('Delta-V best     = %.6f m/s\n', best.metrics.deltaV);
fprintf('Improvement      = %.2f %%\n', 100*(baseFull.metrics.deltaV-best.metrics.deltaV)/baseFull.metrics.deltaV);
fprintf('Min robust margin= %.3f m\n', best.metrics.minRobustMargin);
fprintf('Final pos error  = %.3f m\n', best.metrics.finalPositionError);
fprintf('Post-CA RMS error= %.3f m\n', best.metrics.postEncounterRmsError);
fprintf('Saved MAT: %s\n', matFile);
fprintf('Saved CSV: %s\n', summaryFile);
fprintf('Best weights script: %s\n', bestFile);
fprintf('============================================================\n\n');
end

function limits = makeFunctionalLimits(base, options)
limits.safetyMarginMin = options.SafetyBuffer;
limits.finalErrorMax = max( ...
    base.finalPositionError * (1 + options.TrackingRelativeTolerance), ...
    base.finalPositionError + options.TrackingAbsoluteTolerance);
limits.postRmsErrorMax = max( ...
    base.postEncounterRmsError * (1 + options.TrackingRelativeTolerance), ...
    base.postEncounterRmsError + options.TrackingAbsoluteTolerance);
limits.controlTolerance = 1e-6;
end

function tf = isFunctional(m, limits)
tf = m.finiteSignals && ...
     isfinite(m.deltaV) && ...
     m.minRobustMargin >= limits.safetyMarginMin && ...
     m.finalPositionError <= limits.finalErrorMax && ...
     m.postEncounterRmsError <= limits.postRmsErrorMax && ...
     m.maxControlAbs <= m.controlLimit + limits.controlTolerance;
end

function w = clampWeights(w, base, options)
lo = options.WeightMultiplierMin;
hi = options.WeightMultiplierMax;
w.Q_step = min(max(w.Q_step, base.Q_step*lo), base.Q_step*hi);
w.R_step = min(max(w.R_step, base.R_step*lo), base.R_step*hi);
w.S_step = min(max(w.S_step, base.S_step*lo), base.S_step*hi);
end

function printCase(label, run, functional)
if run.ok
    fprintf('%-14s | dV=%9.5f | robust=%8.2f m | final=%8.2f m | postRMS=%8.2f m | %s\n', ...
        label, run.metrics.deltaV, run.metrics.minRobustMargin, ...
        run.metrics.finalPositionError, run.metrics.postEncounterRmsError, ...
        ternary(functional, 'FUNCTIONAL', 'REJECTED'));
else
    fprintf('%-14s | SIMULATION FAILED | %s\n', label, firstLine(run.message));
end
end

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end

function s = firstLine(message)
parts = splitlines(string(message));
if isempty(parts), s = ""; else, s = parts(1); end
end

function r = emptyRunStruct()
r = struct('ok', false, 'message', "", 'stopTime', nan, ...
    'Q_step', nan(1,6), 'R_step', nan(1,3), 'S_step', nan(1,3), ...
    'metrics', struct(), 'functional', false, 'stage', "");
end

function saveCheckpoint(resultsDir, baseWeights, baseCoarse, baseFull, groupRuns, localRuns, options, fullRuns)
if nargin < 8, fullRuns = repmat(emptyRunStruct(), 0, 1); end
checkpoint = struct('baseWeights',baseWeights, 'baselineCoarse',baseCoarse, ...
    'baselineFull',baseFull, 'groupRuns',groupRuns, 'localRuns',localRuns, ...
    'fullRuns',fullRuns, 'options',options);
save(fullfile(resultsDir, 'mpc_tuning_checkpoint.mat'), 'checkpoint');
end

function T = buildSummaryTable(baseFull, groupRuns, localRuns, fullRuns)
runs = [baseFull; groupRuns; localRuns; fullRuns];
n = numel(runs);
stage = strings(n,1); functional = false(n,1); deltaV = nan(n,1);
minSep = nan(n,1); robust = nan(n,1); finalErr = nan(n,1); postRms = nan(n,1);
q = nan(n,6); r = nan(n,3); s = nan(n,3);
for i=1:n
    stage(i)=runs(i).stage; functional(i)=runs(i).functional;
    if runs(i).ok
        deltaV(i)=runs(i).metrics.deltaV; minSep(i)=runs(i).metrics.minSeparation;
        robust(i)=runs(i).metrics.minRobustMargin; finalErr(i)=runs(i).metrics.finalPositionError;
        postRms(i)=runs(i).metrics.postEncounterRmsError;
    end
    q(i,:)=runs(i).Q_step; r(i,:)=runs(i).R_step; s(i,:)=runs(i).S_step;
end
T = table(stage,functional,deltaV,minSep,robust,finalErr,postRms, ...
    q(:,1),q(:,2),q(:,3),q(:,4),q(:,5),q(:,6), ...
    r(:,1),r(:,2),r(:,3),s(:,1),s(:,2),s(:,3), ...
    'VariableNames', {'stage','functional','deltaV','minSeparation','minRobustMargin', ...
    'finalPositionError','postEncounterRmsError','Q1','Q2','Q3','Q4','Q5','Q6', ...
    'R1','R2','R3','S1','S2','S3'});
end

function writeBestWeights(filename, best, baseline)
fid = fopen(filename, 'w');
assert(fid >= 0, 'Could not create %s', filename);
c = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%% Best functional MPC weights found by tune_mpc_deltaV.m\n');
fprintf(fid, '%% Baseline Delta-V: %.12g m/s\n', baseline.metrics.deltaV);
fprintf(fid, '%% Best Delta-V:     %.12g m/s\n', best.metrics.deltaV);
fprintf(fid, '%% Min robust margin: %.12g m\n', best.metrics.minRobustMargin);
fprintf(fid, 'Q_step = [%s];\n', num2str(best.Q_step, ' %.12g'));
fprintf(fid, 'R_step = [%s];\n', num2str(best.R_step, ' %.12g'));
fprintf(fid, 'S_step = [%s];\n', num2str(best.S_step, ' %.12g'));
end

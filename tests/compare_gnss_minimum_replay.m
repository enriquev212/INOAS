function report = compare_gnss_minimum_replay(filename, stopTime)
% Compare thresholds through the receiver function, without a plant simulation.
% Alarm pulses are deterministic test inputs, not measured pseudo-NIS histories.
data = readmatrix(filename, 'CommentStyle', '#');
assert(size(data, 2) == 17 && all(diff(data(:, 1)) > 0));
assert(stopTime <= data(end, 1));
cfg = inoasMinimalGnssConfig(1, 3);
assert(cfg.nsvMin == 5);
finite = all(isfinite(data(:, [7, 9, 17])), 2);
valid = finite & data(:, 7) >= 0.5 & data(:, 17) > 0 & data(:, 17) <= cfg.pdopMax;
q4 = valid & data(:, 9) >= 4;
q5 = valid & data(:, 9) >= cfg.nsvMin;
report.input = char(filename);
report.records = size(data, 1);
report.recordsWithFourUsedSatellites = nnz(data(:, 9) == 4);
report.qualityDifferences24h = nnz(q4 ~= q5);
report.stopTime_s = stopTime;
report.masterStep_s = cfg.Ts;
report.outputOrder = {'lambda', 'receiver_on', 'mode', 'quality_ok'};

t = (0:cfg.Ts:stopTime)';
scores = zeros(numel(t), 2);
scores((t >= 600 & t < 620) | (t >= 2000 & t < 2020), 2) = 20;
labels = {'no_alarm', 'scripted_alarm_pulses'};
for k = 1:size(scores, 2)
    old = replay(data, t, scores(:, k), cfg, 4);
    current = replay(data, t, scores(:, k), cfg, cfg.nsvMin);
    fresh = mod(t, cfg.fixInterval) == 0;
    oldFixes = old(:, 1) ~= 0 & fresh;
    newFixes = current(:, 1) ~= 0 & fresh;
    result.alarmInput = labels{k};
    result.steps = numel(t);
    result.differingSteps = sum(old ~= current, 1);
    result.acceptedFixDifferences = nnz(oldFixes ~= newFixes);
    result.acceptedFixCount = nnz(newFixes);
    report.receiverCases(k) = result;
end
end

function outputs = replay(data, t, scores, cfg, minimum)
cfg.nsvMin = minimum;
% Match the loader's existing initial-quality padding and ZOH model blocks.
first = find(data(:, 7) >= 0.5 & data(:, 9) >= minimum & data(:, 17) > 0, 1);
if ~isempty(first) && first > 1
    data(1:first-1, [7, 9, 10, 11, 17]) = ...
        repmat(data(first, [7, 9, 10, 11, 17]), first-1, 1);
end
indices = interp1(data(:, 1), (1:size(data, 1))', t, 'previous');
assert(all(isfinite(indices)));
outputs = zeros(numel(t), 4, 'uint8');
clear inoasMinimalGnssStep
for k = 1:numel(t)
    row = data(indices(k), :);
    [lambda, on, mode, quality] = inoasMinimalGnssStep(scores(k), row(9), ...
        row(17), row(10), row(11), row(7), t(k), cfg);
    outputs(k, :) = uint8([lambda, on, double(mode), quality]);
end
clear inoasMinimalGnssStep
end

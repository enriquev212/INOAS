function values = get_logsout_signal(logsout, candidateNames)
% Resolve a logged signal by trying several compatible names.

    if isstring(candidateNames)
        candidateNames = cellstr(candidateNames);
    end

    availableNames = string(logsout.getElementNames);

    for i = 1:numel(candidateNames)
        candidate = string(candidateNames{i});
        if any(availableNames == candidate)
            values = logsout.get(char(candidate)).Values;
            return;
        end
    end

    error("None of the requested logsout signals were found: %s", strjoin(string(candidateNames), ", "));
end

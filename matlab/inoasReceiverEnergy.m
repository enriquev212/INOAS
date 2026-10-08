function [cumulativeWh, poweredFraction, savingFraction] = inoasReceiverEnergy(time, mode)
%INOASRECEIVERENERGY Integrate the held receiver state, not the correction flag.
time = double(time(:));
mode = double(mode(:));
assert(numel(time) == numel(mode) && numel(time) >= 2);
assert(all(isfinite(time)) && all(diff(time) > 0));
assert(all(ismember(mode, [0, 1, 2])));
powerByMode = [0.025; 1.3*1.8; 1.8];
power = powerByMode(mode+1);
dt = diff(time);
cumulativeWh = [0; cumsum(power(1:end-1).*dt)]/3600;
duration = time(end)-time(1);
poweredFraction = sum(double(mode(1:end-1) ~= 0).*dt)/duration;
continuousWh = 1.8*duration/3600;
savingFraction = 1-cumulativeWh(end)/continuousWh;
end

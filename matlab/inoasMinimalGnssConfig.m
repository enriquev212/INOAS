function cfg = inoasMinimalGnssConfig(Ts, fixInterval)
% Manual integration example. Acquisition time is a test assumption.
cfg.Ts = Ts;
cfg.fixInterval = fixInterval;
cfg.fixEpoch = 0;
cfg.acquisitionTime = 35;
cfg.minTrackingTime = 60;
cfg.offTime = 300;
cfg.auxThreshold = 12;
cfg.nsvMin = 4;
cfg.pdopMax = 6;
cfg.hpeMax = 5;
cfg.vpeMax = 5;
assert(Ts > 0 && fixInterval >= Ts);
assert(abs(fixInterval / Ts - round(fixInterval / Ts)) < 1e-10);
end

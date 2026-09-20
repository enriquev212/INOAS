function cfg = inoasMinimalGnssConfig(Ts, fixInterval)
% Manual integration example. Acquisition time is a test assumption.
cfg.Ts = Ts;
cfg.fixInterval = fixInterval;
cfg.fixEpoch = 0;
cfg.acquisitionTime = 35;
cfg.minTrackingTime = 60;
cfg.offTime = 300;
% Auxiliary alarm threshold. The score is now a three-channel quadratic form,
% because the derived altitude channel was removed, so the 12 chosen for four
% channels would drop the false-alarm rate from 1.7 % to 0.7 % and make the
% wake-up branch harder to trigger. 10.3 keeps the original rate at 1.6 %.
cfg.auxThreshold = 10.3;
cfg.nsvMin = 4;
cfg.pdopMax = 6;
assert(Ts > 0 && fixInterval >= Ts);
assert(abs(fixInterval / Ts - round(fixInterval / Ts)) < 1e-10);
end

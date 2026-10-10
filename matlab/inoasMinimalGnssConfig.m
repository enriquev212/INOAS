function cfg = inoasMinimalGnssConfig(Ts, fixInterval)
% Receiver timings and quality gate of the AUX3 model.
cfg.Ts = Ts;
cfg.fixInterval = fixInterval;
cfg.fixEpoch = 0;
cfg.acquisitionTime = 35;
cfg.minTrackingTime = 60;
cfg.offTime = 300;
cfg.fixedOnDuration = 96;
cfg.policy = uint8(2); % Full=0, Fixed-Time=1, Reactive=2.
% Empirical threshold for the smoothed/delayed post-update AUX3 score.
% It is not a chi-square-calibrated pre-update innovation test.
cfg.auxThreshold = 10.3;
% GPS/Galileo: three position components and two receiver clock terms.
% Satellite count alone does not establish full geometry-matrix rank.
cfg.nsvMin = 5;
cfg.pdopMax = 6;
assert(Ts > 0 && fixInterval >= Ts);
assert(abs(fixInterval / Ts - round(fixInterval / Ts)) < 1e-10);
end

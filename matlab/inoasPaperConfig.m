function cfg = inoasPaperConfig(varargin)
%INOASPAPERCONFIG Public run options; physical/filter/MPC tuning stays unchanged.
p = inputParser;
p.addParameter('ReceiverPolicy', 'reactive');
p.addParameter('RadiusMode', 'adaptive');
p.addParameter('StopTime', 6743, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0 && x <= 86390);
p.addParameter('Seed', 42, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0 && x <= 2^32-1 && x == floor(x));
p.addParameter('AcquisitionTime', 35, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
p.addParameter('InitialError', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 6 && all(isfinite(x(:)))));
p.parse(varargin{:});
cfg = p.Results;
cfg.ReceiverPolicy = validatestring(cfg.ReceiverPolicy, {'full', 'fixed', 'reactive'});
cfg.RadiusMode = validatestring(cfg.RadiusMode, {'adaptive', 'constant295'});
end

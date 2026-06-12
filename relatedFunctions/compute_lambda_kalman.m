function lambda_kf = compute_lambda_kalman(Pk, x_kf, x_gnss, gnss_available, lambda_prev)
% COMPUTE_LAMBDA_KALMAN  GNSS reactivation (divergence) logic.
%
% Implements Section 5.2.2 / Section 7.4 of the INOAS report.
%
% When the KF is the primary navigation source (lambda_prev < 0.5) this
% function monitors the health of the autonomous estimate and re-enables
% GNSS whenever the composite severity metric eta_k reaches the threshold
% or any hard-stop condition is triggered.
%
% Severity metric (Eq. 5.17 / 8.1):
%   eta_k = max( sigma_pos / 120,
%                sigma_vel / 50,
%                sigma_pos / sigma_pos_min / 1.5,
%                sigma_vel / sigma_vel_min / 1.5,
%                ||x_gnss - x_kf|| / 1000 )
%
% Hard-stop conditions (Eq. 5.24):
%   - Covariance contains non-finite values
%   - Any diagonal element is non-positive
%   - KF has propagated autonomously for >= N_refresh epochs
%   - (eta_k >= eta_th) AND (nKF >= N_hold_KF)
%
% GNSS is re-enabled after bad-health counter >= N_bad_min epochs.
%
% Inputs:
%   Pk             – 6x6 UKF covariance matrix (ECI frame)
%   x_kf           – 6x1 KF state estimate  [m; m; m; m/s; m/s; m/s]
%   x_gnss         – 6x1 raw GNSS state (position+velocity)
%   gnss_available – scalar flag: 1 if x_gnss is a valid GNSS fix, 0 otherwise
%   lambda_prev    – previous lambda value (fed back from Unit Delay)
%
% Output:
%   lambda_kf – 0: stay in KF-primary mode
%               1: command GNSS reactivation

%--------------------------------------------------------------------------
% Tuning parameters (match report Section 5.2.2 / 7.4)
%--------------------------------------------------------------------------
alpha        = 0.98;   % EMA smoothing factor
sigma_pos_th = 120.0;  % absolute position threshold [m]
sigma_vel_th = 50.0;   % absolute velocity threshold [m/s]
growth_th    = 1.5;    % relative-growth threshold [-]
disc_scale   = 1000.0; % state-discrepancy normalisation [m]
eta_th       = 0.5;    % severity reactivation threshold [-]
N_hold_KF    = 50;     % minimum KF epochs before severity is evaluated
N_refresh    = 2000;   % mandatory refresh interval [epochs]
N_bad_min    = 25;     % bad-health persistence for reactivation [epochs]
eps_floor    = 1e-6;   % numerical floor for running minima

%--------------------------------------------------------------------------
% Persistent state
%--------------------------------------------------------------------------
persistent sigma_pos_smooth sigma_vel_smooth
persistent sigma_pos_min sigma_vel_min
persistent nKF bad_count
if isempty(sigma_pos_smooth); sigma_pos_smooth = 0.0; end
if isempty(sigma_vel_smooth);  sigma_vel_smooth  = 0.0; end
if isempty(sigma_pos_min);     sigma_pos_min     = Inf; end
if isempty(sigma_vel_min);     sigma_vel_min     = Inf; end
if isempty(nKF);               nKF     = int32(0);      end
if isempty(bad_count);         bad_count = int32(0);    end

%--------------------------------------------------------------------------
% Default: stay in KF-primary mode
%--------------------------------------------------------------------------
lambda_kf = 0.0;

% ---- Only run reactivation logic when KF is currently primary -----------
if lambda_prev < 0.5

    % Increment KF-mode counter
    nKF = nKF + int32(1);

    % ------------------------------------------------------------------
    % Extract position / velocity covariance submatrices
    % ------------------------------------------------------------------
    Pr = Pk(1:3, 1:3);
    Pv = Pk(4:6, 4:6);
    Pr = 0.5*(Pr + Pr');
    Pv = 0.5*(Pv + Pv');

    % ------------------------------------------------------------------
    % Hard-stop 1: non-finite covariance
    % ------------------------------------------------------------------
    cov_finite = all(isfinite(Pk(:)));
    diag_ok    = all(diag(Pk) > 0);

    % ------------------------------------------------------------------
    % Raw uncertainty metrics
    % ------------------------------------------------------------------
    tr_pos  = Pr(1,1) + Pr(2,2) + Pr(3,3);
    tr_vel  = Pv(1,1) + Pv(2,2) + Pv(3,3);
    sigma_pos_raw = sqrt(max(tr_pos, 0.0));
    sigma_vel_raw = sqrt(max(tr_vel, 0.0));

    % ------------------------------------------------------------------
    % Exponential moving average smoothing
    % ------------------------------------------------------------------
    sigma_pos_smooth = alpha * sigma_pos_smooth + (1-alpha) * sigma_pos_raw;
    sigma_vel_smooth = alpha * sigma_vel_smooth + (1-alpha) * sigma_vel_raw;

    % Update running minima
    if sigma_pos_smooth < sigma_pos_min
        sigma_pos_min = sigma_pos_smooth;
    end
    if sigma_vel_smooth < sigma_vel_min
        sigma_vel_min = sigma_vel_smooth;
    end

    % ------------------------------------------------------------------
    % State discrepancy sub-metric (only when GNSS heartbeat available)
    % ------------------------------------------------------------------
    if gnss_available > 0.5
        disc = norm(x_gnss - x_kf) / disc_scale;
    else
        disc = 0.0;
    end

    % ------------------------------------------------------------------
    % Composite severity metric  eta_k  (Eq. 5.17 / 8.1)
    % ------------------------------------------------------------------
    m1 = sigma_pos_smooth / sigma_pos_th;
    m2 = sigma_vel_smooth / sigma_vel_th;
    m3 = sigma_pos_smooth / max(sigma_pos_min, eps_floor) / growth_th;
    m4 = sigma_vel_smooth / max(sigma_vel_min, eps_floor) / growth_th;
    eta_k = max([m1, m2, m3, m4, disc]);

    % ------------------------------------------------------------------
    % Evaluate KF-bad condition
    % ------------------------------------------------------------------
    kf_bad = false;

    % Hard-stops (immediate trigger)
    if ~cov_finite || ~diag_ok
        kf_bad = true;
    end

    % Mandatory refresh
    if nKF >= int32(N_refresh)
        kf_bad = true;
    end

    % Severity-based trigger (only after initial hold)
    if nKF >= int32(N_hold_KF) && eta_k >= eta_th
        kf_bad = true;
    end

    % ------------------------------------------------------------------
    % Bad-health counter with hysteresis
    % ------------------------------------------------------------------
    if kf_bad
        bad_count = bad_count + int32(1);
    else
        bad_count = int32(max(double(bad_count) - 1, 0));
    end

    % ------------------------------------------------------------------
    % Trigger reactivation
    % ------------------------------------------------------------------
    if bad_count >= int32(N_bad_min)
        lambda_kf = 1.0;
        % Reset persistent state for fresh GNSS-on cycle
        sigma_pos_smooth = 0.0;
        sigma_vel_smooth = 0.0;
        sigma_pos_min    = Inf;
        sigma_vel_min    = Inf;
        nKF              = int32(0);
        bad_count        = int32(0);
    end

else
    % GNSS is currently on – reset KF-mode state
    sigma_pos_smooth = 0.0;
    sigma_vel_smooth = 0.0;
    sigma_pos_min    = Inf;
    sigma_vel_min    = Inf;
    nKF              = int32(0);
    bad_count        = int32(0);

end  % lambda_prev check
end

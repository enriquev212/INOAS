function lambda_gnss = compute_lambda_gnss(Pk, lambda_prev)
% COMPUTE_LAMBDA_GNSS  GNSS deactivation (convergence) logic.
%
% Implements Section 5.2.2 / Section 7.4 of the INOAS report.
%
% When the GNSS receiver is active (lambda_prev >= 0.5) this function
% monitors UKF convergence and switches GNSS off once:
%   1. The minimum hold window has elapsed  (Nhold  = 60 epochs)
%   2. Position uncertainty is below threshold (sigma_enter = 65 m)
%   3. The above condition persists for N_good_min = 120 consecutive epochs
%
% Inputs:
%   Pk          – 6x6 UKF covariance matrix (ECI frame)
%   lambda_prev – previous lambda value (fed back from Unit Delay)
%
% Output:
%   lambda_gnss – 0: command GNSS OFF (hand over to KF-primary)
%                 1: keep GNSS ON  (unchanged / still converging)
%
% NOTE: This block is only evaluated by the routing switch when
%       lambda_prev >= 0.5.  It always outputs a valid lambda so that
%       Simulink signal dimensions are consistent.

%--------------------------------------------------------------------------
% Tuning parameters  (match report Section 5.2.2 and Section 7.4)
%--------------------------------------------------------------------------
N_hold      = 60;   % minimum stabilisation window [epochs]
sigma_enter = 65;   % convergence threshold [m]
N_good_min  = 120;  % persistence window [epochs]

%--------------------------------------------------------------------------
% Persistent counters
%--------------------------------------------------------------------------
persistent active_count good_count
if isempty(active_count);  active_count = int32(0); end
if isempty(good_count);    good_count   = int32(0); end

%--------------------------------------------------------------------------
% Default: keep GNSS on
%--------------------------------------------------------------------------
lambda_gnss = 1.0;

% ---- Only run deactivation logic when GNSS is currently active ----------
if lambda_prev >= 0.5

    % Count consecutive GNSS-on epochs
    active_count = active_count + int32(1);

    % --- Confidence metric: sqrt(max(trace(Pr), 0)) ---------------------
    Pr = Pk(1:3, 1:3);
    Pr = 0.5 * (Pr + Pr');                     % symmetrise
    tr_val = Pr(1,1) + Pr(2,2) + Pr(3,3);     % trace
    m = sqrt(max(tr_val, 0.0));                % position std [m]

    % --- Check convergence once minimum hold has elapsed ----------------
    if active_count >= int32(N_hold)

        if m <= sigma_enter
            good_count = good_count + int32(1);
        else
            good_count = int32(0);             % reset on any bad epoch
        end

        % Switch GNSS off only after sustained convergence
        if good_count >= int32(N_good_min)
            lambda_gnss = 0.0;
            % Reset counters so the block is fresh on next GNSS-on phase
            active_count = int32(0);
            good_count   = int32(0);
        end

    end  % hold window

else
    % GNSS is currently off – reset counters (will restart on reactivation)
    active_count = int32(0);
    good_count   = int32(0);

end  % lambda_prev check
end

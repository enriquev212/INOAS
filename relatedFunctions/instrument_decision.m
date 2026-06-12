function lambda = instrument_decision(J, n_sat, PDOP, HPE, VPE, gnss_sol)
% INSTRUMENT_DECISION  State machine: GNSS vs Kalman selector
%   Outputs lambda = 1 (GNSS active)  or  0 (Kalman propagation only).
%
% INPUTS
%   J        - trace(S_inv * P * S_T_inv)  covariance observable [scalar]
%   n_sat    - number of GNSS satellites used (NSV)
%   PDOP     - position dilution of precision
%   HPE,VPE  - horizontal / vertical position error [m]
%   gnss_sol - GNSS solution validity flag (1 valid, 0 no-fix)
%
% BEHAVIOUR
%   GNSS   -> KALMAN : timer >= T_gnss_on  (periodic duty cycle)
%                      OR any Table-3.1 emergency condition
%   KALMAN -> GNSS   : timer >= T_kalman_on AND GNSS healthy
%                      OR J > max_cov AND GNSS healthy
%#codegen

persistent state timer_count;

% --- Parameters ----------------------------------------------------------
T_gnss_on = 90;   % [s] GNSS ON period before switching to Kalman
T_kalman_on = 10; % [s] Kalman ON period before returning to GNSS
max_cov   = 2000; % J threshold for forced return to GNSS
n_sat_min = 4;    % minimum satellites for a valid fix  (Table 3.1)
PDOP_max  = 6;    % PDOP limit                          (Table 3.1)
HPE_max   = 5;    % horizontal position error limit [m] (Table 3.1)
VPE_max   = 5;    % vertical   position error limit [m] (Table 3.1)
Ts        = 1;    % [s] execution sample time of this decision block

GNSS_STATE   = int32(0);
KALMAN_STATE = int32(1);

if isempty(state)
    state       = GNSS_STATE;
    timer_count = int32(0);
end

% --- Table-3.1 emergency conditions -------------------------------------
signal_outage   = (n_sat < n_sat_min) || (gnss_sol < 0.5);
precision_loss  = (HPE > HPE_max)     || (VPE > VPE_max);
geometric_drift = (PDOP > PDOP_max);
emergency       = signal_outage || precision_loss || geometric_drift;

timer_count = timer_count + int32(Ts);

switch state
    case GNSS_STATE
        if emergency
            % Immediate forced switch - GNSS is unusable
            state       = KALMAN_STATE;
            timer_count = int32(0);
        elseif timer_count >= int32(T_gnss_on)
            % Periodic duty-cycle: GNSS has been on long enough
            state       = KALMAN_STATE;
            timer_count = int32(0);
        end

    case KALMAN_STATE
        % Real duty-cycle: leave Kalman after its ON window, or earlier if
        % the Kalman covariance observable becomes too large.
        gnss_healthy = ~emergency;
        if gnss_healthy && ((timer_count >= int32(T_kalman_on)) || (J > max_cov))
            state       = GNSS_STATE;
            timer_count = int32(0);
        end

    otherwise
        state       = GNSS_STATE;
        timer_count = int32(0);
end

lambda = double(state == GNSS_STATE);

end

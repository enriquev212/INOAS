function dt = inoas_estimator_dt()
%INOAS_ESTIMATOR_DT Estimator/UKF integration step [s]. Single source of truth.
%
%   The UKF block executes at the Simulink sample time Ts and calls
%   myStateTransitionFcn, which integrates the orbital dynamics over a fixed
%   step. Those two have to be the same number. They used to be two independent
%   literals -- Ts in initialize_inoas_simulation.m and dt = 1 buried in
%   myStateTransitionFcn -- so setting Ts = 0.5 s made the filter propagate
%   twice as fast as the plant: a systematic, same-sign drift of roughly 3.6 km
%   per step at orbital velocity, with no error and no warning.
%
%   It stays a literal rather than reading the base workspace so that the
%   state-transition function remains code-generation compatible. Changing the
%   estimator rate means changing this value AND Ts; the initialization script
%   asserts that they agree.
    dt = 1;
end

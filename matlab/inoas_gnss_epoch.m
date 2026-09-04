function dt = inoas_gnss_epoch()
%INOAS_GNSS_EPOCH GNSS measurement epoch [s]. Single source of truth.
%
%   The GNSS zero-order holds in the model run at gnss_sample_time, set in
%   initialize_inoas_simulation.m, while the gate that enables the UKF
%   measurement update carried its own literal (gnss_rate = 3) with a comment
%   telling the reader to keep the two in sync by hand. They are the same
%   quantity and must not be two numbers.
%
%   It stays a literal rather than reading the base workspace so that the gate
%   remains code-generation compatible. Changing the GNSS cadence means changing
%   this value AND gnss_sample_time; the initialization script asserts that they
%   agree.
    dt = 3;
end

function y = myGnssStateMeasurementFcn(x)
%MYGNSSSTATEMEASUREMENTFCN GNSS observes the full translational state.
%
% State layout: [position_eci; velocity_eci].

    y = x(:);
end

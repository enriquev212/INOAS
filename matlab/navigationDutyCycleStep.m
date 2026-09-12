function [lambda, elapsed] = navigationDutyCycleStep(lambda, elapsed, healthy, ...
    score, onDuration, offDuration, threshold, dt)
%NAVIGATIONDUTYCYCLESTEP One supervisor tick, without persistent state.
%#codegen
elapsed = elapsed + dt;
if lambda
    if ~healthy || elapsed >= onDuration
        lambda = false;
        elapsed = 0;
    end
elseif healthy && (elapsed >= offDuration || score >= threshold)
    lambda = true;
    elapsed = 0;
end
end

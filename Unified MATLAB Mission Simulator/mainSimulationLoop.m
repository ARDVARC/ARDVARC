function [times, trueUasStates, estimatedUasStates, estimatedRgv1Positions, estimatedRgv2Positions] = mainSimulationLoop(rgv1, rgv2, duration, uasStartState)
    % TODO: Add a loop here that can be interupted by sensor readings to
    % update the state extrapolaters
    uasStateExtrapolator = magicUasStateEstimator();
    rgv1PositionExtrapolator = magicRgvPositionEstimator(rgv1);
    rgv2PositionExtrapolator = magicRgvPositionEstimator(rgv2);
    [times, trueUasStates] = simulateUasPhysics(uasStateExtrapolator, rgv1PositionExtrapolator, rgv2PositionExtrapolator, 0, duration, uasStartState);
    
    count = length(times);
    estimatedUasStates = zeros(count,12);
    estimatedRgv1Positions = zeros(count,3);
    estimatedRgv2Positions = zeros(count,3);
    for i = 1:count
        time = times(i);
        trueUasState = trueUasStates(i);
        estimatedUasStates(i, :) = uasStateExtrapolator(time, trueUasState);
        estimatedRgv1Positions(i, :) = rgv1.getStateAtTime(time);
        estimatedRgv2Positions(i, :) = rgv2.getStateAtTime(time);
    end
end
function [times, trueUasStates, estimatedUasStates, estimatedRgv1Positions, estimatedRgv2Positions] = mainSimulationLoop(rgv1, rgv2, duration, uasStartState)
    % Runs a UAS physics simulation loop while periodically updating the
    % estimated state with new sensor data
    % TODO: Currently doesn't actually loop because the sensor models
    % aren't there yet
    arguments(Input)
        rgv1 (1,1) RGV
        rgv2 (1,1) RGV
        duration (1,1) double
        uasStartState (12,1) double
    end
    arguments(Output)
        times (1,:) double
        trueUasStates (:,12) double
        estimatedUasStates (:,12) double
        estimatedRgv1Positions (:,3) double
        estimatedRgv2Positions (:,3) double
    end

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
function [rgvPositions, rgvEulers, rgvMovementTypes] = getRGVstatesAtTimes(rgv, times)
    arguments(Input)
        rgv (1,1) RGV
        times (1,:) double
    end
    arguments(Output)
        rgvPositions (:,3) double
        rgvEulers (:,3) double
        rgvMovementTypes (:,1) int8
    end
    rgvPositions = zeros(length(times),3);
    rgvEulers = zeros(length(times),3);
    rgvMovementTypes = zeros(length(times),1,"int8");
    for i = 1:length(times)
        [rgvPositions(i,:), rgvEulers(i,:), rgvMovementTypes(i)] = rgv.getStateAtTime(times(i));
    end
end
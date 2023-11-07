function [pos, euler, movementType] = getRgvStateAtTime(rgv, time)
    % Determines where the specified RGV would be at the specified time by
    % interpolating between the different known RGV positions according to
    % the known movement patterns
    arguments(Input)
        rgv (1,1) RGV
        time (1,1) double
    end
    arguments(Output)
        pos (1,3) double
        euler (1,3) double
        movementType (1,1) RgvMovementType
    end

    % Find the previous known location of the RGV as well as how it will be
    % moving
    prevStateIndex = find(rgv.times <= time, 1, "last");
    startTime = rgv.times(prevStateIndex);
    startPos = rgv.positions(prevStateIndex, :);
    startEul = rgv.eulers(prevStateIndex, :);
    movementType = rgv.movementTypes(prevStateIndex);

    % Reenact the RGV moving how it did to find out where it would be at
    % the destired time
    [pos, euler] = moveRgv(time, startTime, startPos, startEul, movementType);
end
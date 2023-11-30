function [vec_rgvPos_en, yawAngle, movementType] = getRgvStateAtTime(rgvParams, rgv, time)
    % Determines where the specified RGV would be at the specified time by
    % interpolating between the different known RGV positions according to
    % the known movement patterns
    arguments(Input)
        rgvParams (1,1) RgvParams
        rgv (1,1) RGV
        time (1,1) double
    end
    arguments(Output)
        vec_rgvPos_en (2,1) double
        yawAngle (1,1) double
        movementType (1,1) RgvMovementType
    end

    % Find the previous known location of the RGV as well as how it will be
    % moving
    prevStateIndex = find(rgv.vec_time <= time, 1, "last");
    startTime = rgv.vec_time(prevStateIndex);
    vec_startPos_en = rgv.trix_vec_position_en(:, prevStateIndex);
    startYawAngle = rgv.vec_yawAngle(prevStateIndex);
    movementType = rgv.vec_movementType(prevStateIndex);

    % Reenact the RGV moving how it did to find out where it would be at
    % the destired time
    [vec_rgvPos_en, yawAngle] = moveRgv(rgvParams, time, startTime, vec_startPos_en, startYawAngle, movementType);
end
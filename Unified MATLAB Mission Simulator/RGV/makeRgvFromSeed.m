function this = makeRgvFromSeed(seed, startPos, startEul, duration, params)
    arguments(Input)
        seed (1,1) double
        startPos (1,3) double
        startEul (1,3) double
        duration (1,1) double
        params (1,1) RgvParams
    end
    arguments(Output)
        this (1,1) RGV
    end

    rng(seed);

    vecSize = duration;

    times = zeros(vecSize, 1);
    positions = zeros(vecSize, 3);
    eulers = zeros(vecSize, 3);
    movementTypes = zeros(vecSize, 1, "int8");
    coder.varsize('times');
    coder.varsize('positions');
    coder.varsize('eulers');
    coder.varsize('movementTypes');
    
    times(1) = 0;
    positions(1,:) = startPos;
    eulers(1,:) = startEul;
    movementTypes(1) = getRandomRgvMovementType(params);

    counter = 1;
    time = 0;

    while (time <= duration)
        if (counter + 1 > vecSize)
            vecSize = vecSize * 2;
            timesTemp = zeros(vecSize, 1);
            timesTemp(1:counter) = times;
            times = timesTemp;
            positionsTemp = zeros(vecSize, 3);
            positionsTemp(1:counter,:) = positions;
            positions = positionsTemp;
            eulersTemp = zeros(vecSize, 3);
            eulersTemp(1:counter,:) = eulers;
            eulers = eulersTemp;
            movementTypesTemp = zeros(vecSize, 1, "int8");
            movementTypesTemp(1:counter,:) = movementTypes;
            movementTypes = movementTypesTemp;
        end

        switch movementTypes(counter)
            case RGVMovementType.Wait
                deltaTime = rand(1)*(params.waitTimeMax-params.waitTimeMin)+params.waitTimeMin;
            case RGVMovementType.Straight
                deltaTime = rand(1)*(params.straightTimeMax-params.straightTimeMin)+params.straightTimeMin;
            case { RGVMovementType.ArcLeft, RGVMovementType.ArcRight }
                deltaTime = rand(1)*(params.arcTimeMax-params.arcTimeMin)+params.arcTimeMin;
            case { RGVMovementType.UTurnLeft, RGVMovementType.UTurnRight }
                deltaTime = rand(1)*(params.uTurnTimeMax-params.uTurnTimeMin)+params.uTurnTimeMin;
            otherwise
                deltaTime = rand(1)*params.waitTimeMin+(params.waitTimeMax-params.waitTimeMin);
        end
        [newPos, newEuler] = moveRgv(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter), params);
        if (distanceToBoundary(newPos) < params.safeDistanceFromEdge)
            % opts = optimoptions('Display','off');
            opts = optimoptions("lsqnonlin","Algorithm","levenberg-marquardt",'Display','off');
            deltaTime = lsqnonlin(@(x) distanceToBoundary(moveRgv(time + x, time, positions(counter,:), eulers(counter,:), movementTypes(counter))) - params.safeDistanceFromEdge, deltaTime, 0, inf, opts);
            [newPos, newEuler] = moveRgv(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter), params);
            dir = eul2rotm(newEuler)*[1;0;0];
            if (signedAngle(newPos(1:2), dir(1:2)) > 0)
                movementTypes(counter+1,:) = RGVMovementType.UTurnLeft;
            else
                movementTypes(counter+1,:) = RGVMovementType.UTurnRight;
            end
        else
            movementTypes(counter+1,:) = getRandomRgvMovementType(params);
        end
        counter = counter + 1;
        time = time + deltaTime;
        times(counter) = time;
        positions(counter,:) = newPos;
        eulers(counter,:) = newEuler;
    end

    times = times(1:counter);
    positions = positions(1:counter,:);
    eulers = eulers(1:counter,:);
    movementTypes = movementTypes(1:counter);

    this = RGV(params, times, positions, eulers, movementTypes);
end
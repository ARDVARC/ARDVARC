function this = makeRgvFromSeed(seed, startPos, startEul, duration)
    % Creates a new RGV with the specified starting conditions by moving it
    % in large steps according to seeded random movement types. Generates
    % movements until the specified duration is reached.
    arguments(Input)
        seed (1,1) double
        startPos (1,3) double
        startEul (1,3) double
        duration (1,1) double
    end
    arguments(Output)
        this (1,1) RGV
    end

    global simParams;

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
    movementTypes(1) = getRandomRgvMovementType();

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
            case RgvMovementType.Wait
                deltaTime = rand(1)*(simParams.rgvParams.waitTimeMax-simParams.rgvParams.waitTimeMin)+simParams.rgvParams.waitTimeMin;
            case RgvMovementType.Straight
                deltaTime = rand(1)*(simParams.rgvParams.straightTimeMax-simParams.rgvParams.straightTimeMin)+simParams.rgvParams.straightTimeMin;
            case { RgvMovementType.ArcLeft, RgvMovementType.ArcRight }
                deltaTime = rand(1)*(simParams.rgvParams.arcTimeMax-simParams.rgvParams.arcTimeMin)+simParams.rgvParams.arcTimeMin;
            case { RgvMovementType.UTurnLeft, RgvMovementType.UTurnRight }
                deltaTime = rand(1)*(simParams.rgvParams.uTurnTimeMax-simParams.rgvParams.uTurnTimeMin)+simParams.rgvParams.uTurnTimeMin;
            otherwise
                deltaTime = rand(1)*simParams.rgvParams.waitTimeMin+(simParams.rgvParams.waitTimeMax-simParams.rgvParams.waitTimeMin);
        end
        [newPos, newEuler] = moveRgv(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter));
        if (distanceToBoundary(newPos) < simParams.rgvParams.safeDistanceFromEdge)
            % opts = optimoptions('Display','off');
            opts = optimoptions("lsqnonlin","Algorithm","levenberg-marquardt",'Display','off');
            deltaTime = lsqnonlin(@(x) distanceToBoundary(moveRgv(time + x, time, positions(counter,:), eulers(counter,:), movementTypes(counter))) - simParams.rgvParams.safeDistanceFromEdge, deltaTime, 0, inf, opts);
            [newPos, newEuler] = moveRgv(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter));
            dir = eul2rotm(newEuler)*[1;0;0];
            if (signedAngle(newPos(1:2), dir(1:2)) > 0)
                movementTypes(counter+1,:) = RgvMovementType.UTurnLeft;
            else
                movementTypes(counter+1,:) = RgvMovementType.UTurnRight;
            end
        else
            movementTypes(counter+1,:) = getRandomRgvMovementType();
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

    this = RGV(times, positions, eulers, movementTypes);
end
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

    % Set the seed of the random number generator so that the RGV
    % generation is consistent
    rng(seed);

    % Preallocation to make MATLAB Coder happy
    vecSize = duration;
    times = zeros(vecSize, 1);
    positions = zeros(vecSize, 3);
    eulers = zeros(vecSize, 3);
    movementTypes = zeros(vecSize, 1, "int8");
    coder.varsize('times');
    coder.varsize('positions');
    coder.varsize('eulers');
    coder.varsize('movementTypes');
    
    % Set starting conditions
    times(1) = 0;
    positions(1,:) = startPos;
    eulers(1,:) = startEul;
    movementTypes(1) = getRandomRgvMovementType();

    counter = 1;
    time = 0;

    while (time <= duration)
        % If we haven't reached the end time, take another step
        if (counter + 1 > vecSize)
            % This is weird allocation code to make sure that the various
            % arrays don't run out of space to store all the RGV states
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

        % Determine how long this step takes based on the known movement
        % type for the step and the known amounts of time that that type of
        % movement can take
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

        % Take a step
        [newPos, newEuler] = moveRgv(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter));
        if (distanceToBoundary(newPos) < simParams.rgvParams.safeDistanceFromEdge)
            % If the step would take the RGV out of the safe region, don't
            % let it. Instead, only move until you hit the safe region,
            % then note that the next step needs to be some kind of u-turn
            opts = optimoptions("lsqnonlin","Algorithm","levenberg-marquardt",'Display','off');
            deltaTime = lsqnonlin(@(x) distanceToBoundary(moveRgv(time + x, time, positions(counter,:), eulers(counter,:), movementTypes(counter))) - simParams.rgvParams.safeDistanceFromEdge, deltaTime, 0, inf, opts);
            [newPos, newEuler] = moveRgv(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter));
            % Choose the u-turn direction based on whichever way will point
            % you closer back to the center of the mission area
            dir = eul2rotm(newEuler)*[1;0;0];
            if (signedAngle(newPos(1:2), dir(1:2)) > 0)
                movementTypes(counter+1,:) = RgvMovementType.UTurnLeft;
            else
                movementTypes(counter+1,:) = RgvMovementType.UTurnRight;
            end
        else
            % If the step wouldn't take the RGV out of the safe region,
            % keep the full step and choose a random movement type for the
            % next step
            movementTypes(counter+1,:) = getRandomRgvMovementType();
        end

        % Update state arrays
        counter = counter + 1;
        time = time + deltaTime;
        times(counter) = time;
        positions(counter,:) = newPos;
        eulers(counter,:) = newEuler;
    end

    % Remove empty ends of state arrays caused by preallocation
    times = times(1:counter);
    positions = positions(1:counter,:);
    eulers = eulers(1:counter,:);
    movementTypes = movementTypes(1:counter);

    this = RGV(times, positions, eulers, movementTypes);
end
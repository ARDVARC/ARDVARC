function this = makeRgvFromSeed(rgvParams, seed, vec_startPos_en, startYawAngle, duration, missionAreaHalfWidth)
    % Creates a new RGV with the specified starting conditions by moving it
    % in large steps according to seeded random movement types. Generates
    % movements until the specified duration is reached.
    arguments(Input)
        rgvParams (1,1) RgvParams
        seed (1,1) double
        vec_startPos_en (2,1) double
        startYawAngle (1,1) double
        duration (1,1) double
        missionAreaHalfWidth (1,1) double
    end
    arguments(Output)
        this (1,1) RGV
    end

    % Set the seed of the random number generator so that the RGV
    % generation is consistent
    rng(seed);

    % Preallocation to make MATLAB Coder happy
    vecSize = duration;
    vec_time = zeros(1, vecSize);
    trix_vec_position_en = zeros(2, vecSize);
    vec_yawAngle = zeros(1, vecSize);
    vec_movementType = zeros(1, vecSize, "int8");
    coder.varsize('times');
    coder.varsize('positions');
    coder.varsize('eulers');
    coder.varsize('movementTypes');
    
    % Set starting conditions
    vec_time(1) = 0;
    trix_vec_position_en(:,1) = vec_startPos_en;
    vec_yawAngle(1) = startYawAngle;
    vec_movementType(1) = getRandomRgvMovementType(rgvParams);

    counter = 1;
    time = 0;

    while (time <= duration)
        % If we haven't reached the end time, take another step
        if (counter + 1 > vecSize)
            % This is weird allocation code to make sure that the various
            % arrays don't run out of space to store all the RGV states
            vecSize = vecSize * 2;
            vec_timeTemp = zeros(1, vecSize);
            vec_timeTemp(1:counter) = vec_time;
            vec_time = vec_timeTemp;
            trix_vec_position_enTemp = zeros(2, vecSize);
            trix_vec_position_enTemp(:,1:counter) = trix_vec_position_en;
            trix_vec_position_en = trix_vec_position_enTemp;
            vec_yawAngleTemp = zeros(1, vecSize);
            vec_yawAngleTemp(1:counter) = vec_yawAngle;
            vec_yawAngle = vec_yawAngleTemp;
            vec_movementTypeTemp = zeros(1, vecSize, "int8");
            vec_movementTypeTemp(1:counter) = vec_movementType;
            vec_movementType = vec_movementTypeTemp;
        end

        % Determine how long this step takes based on the known movement
        % type for the step and the known amounts of time that that type of
        % movement can take
        switch vec_movementType(counter)
            case RgvMovementType.Wait
                deltaTime = rand(1)*(rgvParams.waitTimeMax-rgvParams.waitTimeMin)+rgvParams.waitTimeMin;
            case RgvMovementType.Straight
                deltaTime = rand(1)*(rgvParams.straightTimeMax-rgvParams.straightTimeMin)+rgvParams.straightTimeMin;
            case { RgvMovementType.ArcLeft, RgvMovementType.ArcRight }
                deltaTime = rand(1)*(rgvParams.arcTimeMax-rgvParams.arcTimeMin)+rgvParams.arcTimeMin;
            case { RgvMovementType.UTurnLeft, RgvMovementType.UTurnRight }
                deltaTime = rand(1)*(rgvParams.uTurnTimeMax-rgvParams.uTurnTimeMin)+rgvParams.uTurnTimeMin;
            otherwise
                deltaTime = rand(1)*rgvParams.waitTimeMin+(rgvParams.waitTimeMax-rgvParams.waitTimeMin);
        end

        % Take a step
        [vec_newPos_en, newYawAngle] = moveRgv(rgvParams, time + deltaTime, time, trix_vec_position_en(:,counter), vec_yawAngle(counter), vec_movementType(counter));
        if (distanceToBoundary(missionAreaHalfWidth, vec_newPos_en) < rgvParams.safeDistanceFromEdge)
            % If the step would take the RGV out of the safe region, don't
            % let it. Instead, only move until you hit the safe region,
            % then note that the next step needs to be some kind of u-turn
            opts = optimoptions("lsqnonlin","Algorithm","levenberg-marquardt",'Display','off');
            deltaTime = lsqnonlin(@(x) distanceToBoundary(missionAreaHalfWidth, moveRgv(rgvParams, time + x, time, trix_vec_position_en(:,counter), vec_yawAngle(counter), vec_movementType(counter))) - rgvParams.safeDistanceFromEdge, deltaTime, 0, inf, opts);
            [vec_newPos_en, newYawAngle] = moveRgv(rgvParams, time + deltaTime, time, trix_vec_position_en(:,counter), vec_yawAngle(counter), vec_movementType(counter));
            % Choose the u-turn direction based on whichever way will point
            % you closer back to the center of the mission area
            vec_dir_en = [cos(newYawAngle);sin(newYawAngle)];
            if (signedAngle(vec_newPos_en, vec_dir_en) > 0)
                vec_movementType(counter+1) = RgvMovementType.UTurnLeft;
            else
                vec_movementType(counter+1) = RgvMovementType.UTurnRight;
            end
        else
            % If the step wouldn't take the RGV out of the safe region,
            % keep the full step and choose a random movement type for the
            % next step
            vec_movementType(counter+1) = getRandomRgvMovementType(rgvParams);
        end

        % Update state arrays
        counter = counter + 1;
        time = time + deltaTime;
        vec_time(counter) = time;
        trix_vec_position_en(:,counter) = vec_newPos_en;
        vec_yawAngle(counter) = newYawAngle;
    end

    % Remove empty ends of state arrays caused by preallocation
    vec_time = vec_time(1:counter);
    trix_vec_position_en = trix_vec_position_en(:,1:counter);
    vec_yawAngle = vec_yawAngle(1:counter);
    vec_movementType = vec_movementType(1:counter);

    this = RGV(vec_time, trix_vec_position_en, vec_yawAngle, vec_movementType);
end
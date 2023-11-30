function [vec_newPos_en, newYawAngle] = moveRgv(rgvParams, time, startTime, vec_startPos_en, startYawAngle, movementType)
    % Moves an RGV from some specified starting state for some specified
    % amount of time following a specfied movement type. Returns the final
    % state of the moved RGV.
    arguments(Input)
        rgvParams (1,1) RgvParams
        time (1,1) double
        startTime (1,1) double
        vec_startPos_en (2,1) double
        startYawAngle (1,1) double
        movementType (1,1) RgvMovementType
    end
    arguments(Output)
        vec_newPos_en (2,1) double
        newYawAngle (1,1) double
    end

    switch(movementType)
        case RgvMovementType.Straight
            progressTime = time - startTime;
            vec_dir_en = [cos(startYawAngle);sin(startYawAngle)];
            vec_newPos_en = vec_startPos_en + rgvParams.speed * progressTime * vec_dir_en;
            newYawAngle = startYawAngle;
        case RgvMovementType.ArcLeft
            vec_radiusVectorIn_en = rgvParams.turningRadius * [-sin(startYawAngle);cos(startYawAngle)];
            progressTime = time - startTime;
            addedYawAngle = rgvParams.turningSpeed * progressTime;
            newYawAngle = startYawAngle + addedYawAngle;
            vec_radiusVectorOut_en = rgvParams.turningRadius * [sin(newYawAngle);-cos(newYawAngle)];
            vec_newPos_en = vec_startPos_en + vec_radiusVectorIn_en + vec_radiusVectorOut_en;
        case RgvMovementType.ArcRight
            vec_radiusVectorIn_en = rgvParams.turningRadius * [sin(startYawAngle);-cos(startYawAngle)];
            progressTime = time - startTime;
            addedYawAngle = -rgvParams.turningSpeed * progressTime;
            newYawAngle = startYawAngle + addedYawAngle;
            vec_radiusVectorOut_en = rgvParams.turningRadius * [-sin(newYawAngle);cos(newYawAngle)];
            vec_newPos_en = vec_startPos_en + vec_radiusVectorIn_en + vec_radiusVectorOut_en;
        case RgvMovementType.UTurnLeft
            % U-turn movement has two parts
            vec_radiusVectorIn_en = rgvParams.uTurnRadius * [-sin(startYawAngle);cos(startYawAngle)];
            progressTime = time - startTime;
            if (progressTime < rgvParams.uTurnTurnTime)
                % The first part is an arc
                addedYawAngle = rgvParams.uTurnSpeed  * progressTime;
                newYawAngle = startYawAngle + addedYawAngle;
                vec_radiusVectorOut_en = rgvParams.uTurnRadius * [sin(newYawAngle);-cos(newYawAngle)];
                vec_newPos_en = vec_startPos_en + vec_radiusVectorIn_en + vec_radiusVectorOut_en;
            else
                % The second part is straight
                vec_startPos_en = vec_startPos_en + 2*vec_radiusVectorIn_en;
                newYawAngle = startYawAngle + pi;
                progressTime = progressTime - rgvParams.uTurnTurnTime;
                vec_dir_en = [cos(newYawAngle);sin(newYawAngle)];
                vec_newPos_en = vec_startPos_en + rgvParams.speed * progressTime * vec_dir_en;
            end
        case RgvMovementType.UTurnRight
            % U-turn movement has two parts
            vec_radiusVectorIn_en = rgvParams.uTurnRadius * [sin(startYawAngle);-cos(startYawAngle)];
            progressTime = time - startTime;
            if (progressTime < rgvParams.uTurnTurnTime)
                % The first part is an arc
                addedYawAngle = rgvParams.uTurnSpeed * progressTime;
                newYawAngle = startYawAngle - addedYawAngle;
                vec_radiusVectorOut_en = rgvParams.uTurnRadius * [-sin(newYawAngle);cos(newYawAngle)];
                vec_newPos_en = vec_startPos_en + vec_radiusVectorIn_en + vec_radiusVectorOut_en;
            else
                % The second part is straight
                vec_startPos_en = vec_startPos_en + 2*vec_radiusVectorIn_en;
                newYawAngle = startYawAngle + pi;
                progressTime = progressTime - rgvParams.uTurnTurnTime;
                vec_dir_en = [cos(newYawAngle);sin(newYawAngle)];
                vec_newPos_en = vec_startPos_en + rgvParams.speed * progressTime * vec_dir_en;
            end
        otherwise
            % This case is for "Waiting" or if another movement type is
            % added later but not implemented (it will just stand still)
            vec_newPos_en = vec_startPos_en;
            newYawAngle = startYawAngle;
    end
end
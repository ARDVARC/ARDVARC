function [pos, euler] = moveRgv(time, startTime, startPos, startEul, movementType)
    % Moves an RGV from some specified starting state for some specified
    % amount of time following a specfied movement type. Returns the final
    % state of the moved RGV.
    arguments(Input)
        time (1,1) double
        startTime (1,1) double
        startPos (1,3) double
        startEul (1,3) double
        movementType (1,1) RgvMovementType
    end
    arguments(Output)
        pos (1,3) double
        euler (1,3) double
    end

    global simParams;

    switch(movementType)
        case RgvMovementType.Straight
            progressTime = time - startTime;
            dir = (eul2rotm(startEul) * [1;0;0])';
            pos = startPos + simParams.rgvParams.speed * progressTime * dir;
            euler = startEul;
        case RgvMovementType.ArcLeft
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;simParams.rgvParams.turningRadius;0])';
            progressTime = time - startTime;
            ang = simParams.rgvParams.turningSpeed * progressTime;
            angRotm = axang2rotm([0 0 1 ang]);
            euler = rotm2eul(angRotm*startRotm);
            pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
        case RgvMovementType.ArcRight
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;-simParams.rgvParams.turningRadius;0])';
            progressTime = time - startTime;
            ang = simParams.rgvParams.turningSpeed * progressTime;
            angRotm = axang2rotm([0 0 1 -ang]);
            euler = rotm2eul(angRotm*startRotm);
            pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
        case RgvMovementType.UTurnLeft
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;simParams.rgvParams.uTurnRadius;0])';
            progressTime = time - startTime;
            if (progressTime < simParams.rgvParams.uTurnTurnTime)
                ang = simParams.rgvParams.uTurnSpeed * progressTime;
                angRotm = axang2rotm([0 0 1 ang]);
                euler = rotm2eul(angRotm*startRotm);
                pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
            else
                angRotm = axang2rotm([0 0 1 pi]);
                startRotm = angRotm*startRotm;
                startPos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
    
                progressTime = progressTime - simParams.rgvParams.uTurnTurnTime;
                dir = (startRotm * [1;0;0])';
                pos = startPos + simParams.rgvParams.speed * progressTime * dir;
                euler = rotm2eul(startRotm);
            end
        case RgvMovementType.UTurnRight
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;-simParams.rgvParams.uTurnRadius;0])';
            progressTime = time - startTime;
            if (progressTime < simParams.rgvParams.uTurnTurnTime)
                ang = simParams.rgvParams.uTurnSpeed * progressTime;
                angRotm = axang2rotm([0 0 1 -ang]);
                euler = rotm2eul(angRotm*startRotm);
                pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
            else
                angRotm = axang2rotm([0 0 1 -pi]);
                startRotm = angRotm*startRotm;
                startPos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
    
                progressTime = progressTime - simParams.rgvParams.uTurnTurnTime;
                dir = (startRotm * [1;0;0])';
                pos = startPos + simParams.rgvParams.speed * progressTime * dir;
                euler = rotm2eul(startRotm);
            end
        otherwise
            pos = startPos;
            euler = startEul;
    end
end
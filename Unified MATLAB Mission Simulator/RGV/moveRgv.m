function [pos, euler] = moveRgv(time, startTime, startPos, startEul, movementType, params)
    arguments(Input)
        time (1,1) double
        startTime (1,1) double
        startPos (1,3) double
        startEul (1,3) double
        movementType (1,1) RGVMovementType
        params (1,1) RgvParams
    end
    arguments(Output)
        pos (1,3) double
        euler (1,3) double
    end
    switch(movementType)
        case RGVMovementType.Straight
            progressTime = time - startTime;
            dir = (eul2rotm(startEul) * [1;0;0])';
            pos = startPos + params.speed * progressTime * dir;
            euler = startEul;
        case RGVMovementType.ArcLeft
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;params.turningRadius;0])';
            progressTime = time - startTime;
            ang = params.turningSpeed * progressTime;
            angRotm = axang2rotm([0 0 1 ang]);
            euler = rotm2eul(angRotm*startRotm);
            pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
        case RGVMovementType.ArcRight
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;-params.turningRadius;0])';
            progressTime = time - startTime;
            ang = params.turningSpeed * progressTime;
            angRotm = axang2rotm([0 0 1 -ang]);
            euler = rotm2eul(angRotm*startRotm);
            pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
        case RGVMovementType.UTurnLeft
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;params.uTurnRadius;0])';
            progressTime = time - startTime;
            if (progressTime < params.uTurnTurnTime)
                ang = params.uTurnSpeed * progressTime;
                angRotm = axang2rotm([0 0 1 ang]);
                euler = rotm2eul(angRotm*startRotm);
                pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
            else
                angRotm = axang2rotm([0 0 1 pi]);
                startRotm = angRotm*startRotm;
                startPos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
    
                progressTime = progressTime - params.uTurnTurnTime;
                dir = (startRotm * [1;0;0])';
                pos = startPos + params.speed * progressTime * dir;
                euler = rotm2eul(startRotm);
            end
        case RGVMovementType.UTurnRight
            startRotm = eul2rotm(startEul);
            radiusVectorIn = (startRotm * [0;-params.uTurnRadius;0])';
            progressTime = time - startTime;
            if (progressTime < params.uTurnTurnTime)
                ang = params.uTurnSpeed * progressTime;
                angRotm = axang2rotm([0 0 1 -ang]);
                euler = rotm2eul(angRotm*startRotm);
                pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
            else
                angRotm = axang2rotm([0 0 1 -pi]);
                startRotm = angRotm*startRotm;
                startPos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
    
                progressTime = progressTime - params.uTurnTurnTime;
                dir = (startRotm * [1;0;0])';
                pos = startPos + params.speed * progressTime * dir;
                euler = rotm2eul(startRotm);
            end
        otherwise
            pos = startPos;
            euler = startEul;
    end
end
classdef RGV
    properties(Constant)
        speed (1,1) double = 0.5;
        turningRadius (1,1) double = 4;
        uTurnRadius (1,1) double = 2;
        waitTimeMin (1,1) double = 90;
        waitTimeMax (1,1) double = 100;
        straightTimeMin (1,1) double = 1;
        straightTimeMax (1,1) double = 2;
        arcTimeMin (1,1) double = 2;
        arcTimeMax (1,1) double = 3;
        waitChanceWeight = 0.1;
        straightChanceWeight = 5;
        arcLeftChanceWeight = 1;
        arcRightChanceWeight = 1;
        
        safeDistanceFromEdge = RGV.uTurnRadius*2;

        % Precalculate
        turningSpeed (1,1) double =  RGV.speed / RGV.turningRadius;
        uTurnSpeed (1,1) double =  RGV.speed / RGV.uTurnRadius;
        uTurnTurnTime (1,1) double = pi / RGV.uTurnSpeed;
        uTurnStraightMinTime (1,1) double = 2 * RGV.uTurnRadius / RGV.speed;
        uTurnTimeMin (1,1) double = RGV.uTurnTurnTime + RGV.uTurnStraightMinTime;
        uTurnTimeMax (1,1) double = RGV.uTurnTurnTime + RGV.uTurnStraightMinTime + 4;

        weightSum = RGV.waitChanceWeight + RGV.straightChanceWeight + RGV.arcLeftChanceWeight + RGV.arcRightChanceWeight;
        waitProbCutoff = RGV.waitChanceWeight / RGV.weightSum;
        straightProbCutoff = (RGV.waitChanceWeight + RGV.straightChanceWeight) / RGV.weightSum;
        arcLeftCutoff = (RGV.waitChanceWeight + RGV.straightChanceWeight + RGV.arcLeftChanceWeight) / RGV.weightSum;
    end

    properties
        times (:,1) double
        positions (:,3) double
        eulers (:,3) double
        movementTypes (:,1) RGVMovementType
    end

    methods(Access = private)
        function this = RGV(times, positions, eulers, movementTypes)
            this.times = times;
            this.positions = positions;
            this.eulers = eulers;
            this.movementTypes = movementTypes;
        end
    end

    methods(Access = private, Static)
        function [pos, euler] = move(time, startTime, startPos, startEul, movementType)
            switch(movementType)
                case RGVMovementType.Straight
                    progressTime = time - startTime;
                    dir = (eul2rotm(startEul) * [1;0;0])';
                    pos = startPos + RGV.speed * progressTime * dir;
                    euler = startEul;
                case RGVMovementType.ArcLeft
                    startRotm = eul2rotm(startEul);
                    radiusVectorIn = (startRotm * [0;RGV.turningRadius;0])';
                    progressTime = time - startTime;
                    ang = RGV.turningSpeed * progressTime;
                    angRotm = axang2rotm([0 0 1 ang]);
                    euler = rotm2eul(angRotm*startRotm);
                    pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
                case RGVMovementType.ArcRight
                    startRotm = eul2rotm(startEul);
                    radiusVectorIn = (startRotm * [0;-RGV.turningRadius;0])';
                    progressTime = time - startTime;
                    ang = RGV.turningSpeed * progressTime;
                    angRotm = axang2rotm([0 0 1 -ang]);
                    euler = rotm2eul(angRotm*startRotm);
                    pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
                case RGVMovementType.UTurnLeft
                    startRotm = eul2rotm(startEul);
                    radiusVectorIn = (startRotm * [0;RGV.uTurnRadius;0])';
                    progressTime = time - startTime;
                    if (progressTime < RGV.uTurnTurnTime)
                        ang = RGV.uTurnSpeed * progressTime;
                        angRotm = axang2rotm([0 0 1 ang]);
                        euler = rotm2eul(angRotm*startRotm);
                        pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
                    else
                        angRotm = axang2rotm([0 0 1 pi]);
                        startRotm = angRotm*startRotm;
                        startPos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';

                        progressTime = progressTime - RGV.uTurnTurnTime;
                        dir = (startRotm * [1;0;0])';
                        pos = startPos + RGV.speed * progressTime * dir;
                        euler = rotm2eul(startRotm);
                    end
                case RGVMovementType.UTurnRight
                    startRotm = eul2rotm(startEul);
                    radiusVectorIn = (startRotm * [0;-RGV.uTurnRadius;0])';
                    progressTime = time - startTime;
                    if (progressTime < RGV.uTurnTurnTime)
                        ang = RGV.uTurnSpeed * progressTime;
                        angRotm = axang2rotm([0 0 1 -ang]);
                        euler = rotm2eul(angRotm*startRotm);
                        pos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
                    else
                        angRotm = axang2rotm([0 0 1 -pi]);
                        startRotm = angRotm*startRotm;
                        startPos = startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';

                        progressTime = progressTime - RGV.uTurnTurnTime;
                        dir = (startRotm * [1;0;0])';
                        pos = startPos + RGV.speed * progressTime * dir;
                        euler = rotm2eul(startRotm);
                    end
                otherwise
                    pos = startPos;
                    euler = startEul;
            end
        end
    end

    methods
        function [pos, euler, movementType] = getStateAtTime(this, time)
            prevStateIndex = find(this.times <= time, 1, "last");
            startTime = this.times(prevStateIndex);
            startPos = this.positions(prevStateIndex, :);
            startEul = this.eulers(prevStateIndex, :);
            movementType = this.movementTypes(prevStateIndex);

            [pos, euler] = RGV.move(time, startTime, startPos, startEul, movementType);
        end
    end

    methods(Static)
        function this = makeFromSeed(seed, startPos, startEul, duration)
            arguments(Input)
                seed (1,1) double
                startPos (1,3) double
                startEul (1,3) double
                duration (1,1) double
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

            times(1) = 0;
            positions(1,:) = startPos;
            eulers(1,:) = startEul;
            movementTypes(1) = RGV.getRandomMovementType();

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
                        deltaTime = rand(1)*(RGV.waitTimeMax-RGV.waitTimeMin)+RGV.waitTimeMin;
                    case RGVMovementType.Straight
                        deltaTime = rand(1)*(RGV.straightTimeMax-RGV.straightTimeMin)+RGV.straightTimeMin;
                    case { RGVMovementType.ArcLeft, RGVMovementType.ArcRight }
                        deltaTime = rand(1)*(RGV.arcTimeMax-RGV.arcTimeMin)+RGV.arcTimeMin;
                    case { RGVMovementType.UTurnLeft, RGVMovementType.UTurnRight }
                        deltaTime = rand(1)*(RGV.uTurnTimeMax-RGV.uTurnTimeMin)+RGV.uTurnTimeMin;
                    otherwise
                        deltaTime = rand(1)*RGV.waitTimeMin+(RGV.waitTimeMax-RGV.waitTimeMin);
                end
                [newPos, newEuler] = RGV.move(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter));
                if (Simulation.DistanceToBondary(newPos) < RGV.safeDistanceFromEdge)
                    % opts = optimoptions('Display','off');
                    opts = optimoptions("lsqnonlin","Algorithm","levenberg-marquardt",'Display','off');
                    deltaTime = lsqnonlin(@(x) Simulation.DistanceToBondary(RGV.move(time + x, time, positions(counter,:), eulers(counter,:), movementTypes(counter))) - RGV.safeDistanceFromEdge, deltaTime, 0, inf, opts);
                    [newPos, newEuler] = RGV.move(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter));
                    dir = eul2rotm(newEuler)*[1;0;0];
                    if (signedAngle(newPos, dir) > 0)
                        movementTypes(counter+1,:) = RGVMovementType.UTurnLeft;
                        % wallArcLeftChanceWeight = 3;
                        % wallArcRightChanceWeight = 0;
                        % wallUTurnLeftChanceWeight = .3;
                        % wallUTurnRightChanceWeight = 0;
                    else
                        movementTypes(counter+1,:) = RGVMovementType.UTurnRight;
                        % wallArcLeftChanceWeight = 0;
                        % wallArcRightChanceWeight = 3;
                        % wallUTurnLeftChanceWeight = 0;
                        % wallUTurnRightChanceWeight = .3;
                    end
                    % wallWaitChanceWeight = 0.01;
                    % wallStraightChanceWeight = 1;
                    % wallWeightSum = wallWaitChanceWeight + wallStraightChanceWeight + wallArcLeftChanceWeight + wallArcRightChanceWeight + wallUTurnLeftChanceWeight + wallUTurnRightChanceWeight;
                    % wallWaitProbCutoff = wallWaitChanceWeight / wallWeightSum;
                    % wallStraightProbCutoff = (wallWaitChanceWeight + wallStraightChanceWeight) / wallWeightSum;
                    % wallArcLeftCutoff = (wallWaitChanceWeight + wallStraightChanceWeight + wallArcLeftChanceWeight) / wallWeightSum;
                    % wallUTurnLeftCutoff = (wallWaitChanceWeight + wallStraightChanceWeight + wallArcLeftChanceWeight + wallUTurnLeftChanceWeight) / wallWeightSum;
                    % wallArcRightCutoff = (wallWaitChanceWeight + wallStraightChanceWeight + wallArcLeftChanceWeight + wallUTurnLeftChanceWeight + wallArcRightChanceWeight) / wallWeightSum;
                    % 
                    % randVal = rand(1);
                    % if (randVal < wallWaitProbCutoff)
                    %     movementTypes(counter+1,:) = RGVMovementType.Wait;
                    % elseif (randVal < wallStraightProbCutoff)
                    %     movementTypes(counter+1,:) = RGVMovementType.Straight;
                    % elseif (randVal < wallArcLeftCutoff)
                    %     movementTypes(counter+1,:) = RGVMovementType.ArcLeft;
                    % elseif (randVal < wallUTurnLeftCutoff)
                    %     movementTypes(counter+1,:) = RGVMovementType.UTurnLeft;
                    % elseif (randVal < wallArcRightCutoff)
                    %     movementTypes(counter+1,:) = RGVMovementType.ArcRight;
                    % else
                    %     movementTypes(counter+1,:) = RGVMovementType.UTurnRight;
                    % end

                else
                    movementTypes(counter+1,:) = RGV.getRandomMovementType();
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
        
        function this = getRandomMovementType()
            randVal = rand(1);
            if (randVal < RGV.waitProbCutoff)
                this = RGVMovementType.Wait;
            elseif (randVal < RGV.straightProbCutoff)
                this = RGVMovementType.Straight;
            elseif (randVal < RGV.arcLeftCutoff)
                this = RGVMovementType.ArcLeft;
            else
                this = RGVMovementType.ArcRight;
            end
        end
    end
end
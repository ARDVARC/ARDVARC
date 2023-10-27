classdef RGV
    properties(Constant)
        speed (1,1) double = 0.5;
        turningRadius (1,1) double = 2;
        waitTimeMin (1,1) double = 5;
        waitTimeMax (1,1) double = 10;
        straightTimeMin (1,1) double = 5;
        straightTimeMax (1,1) double = 10;
        arcTimeMin (1,1) double = 2;
        arcTimeMax (1,1) double = 4;

        safeDistanceFromEdge = RGV.turningRadius*2;

        turningSpeed (1,1) double =  RGV.speed / RGV.turningRadius;
        uTurnTurnTime (1,1) double = pi / RGV.turningSpeed;
        uTurnStraightMinTime (1,1) double = 2 * RGV.turningRadius / RGV.speed;
        uTurnTimeMin (1,1) double = RGV.uTurnTurnTime + RGV.uTurnStraightMinTime;
        uTurnTimeMax (1,1) double = RGV.uTurnTurnTime + RGV.uTurnStraightMinTime + 4;
    end

    properties(Access = private)
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
                    radiusVectorIn = (startRotm * [0;RGV.turningRadius;0])';
                    progressTime = time - startTime;
                    if (progressTime < RGV.uTurnTurnTime)
                        ang = RGV.turningSpeed * progressTime;
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
                    radiusVectorIn = (startRotm * [0;-RGV.turningRadius;0])';
                    progressTime = time - startTime;
                    if (progressTime < RGV.uTurnTurnTime)
                        ang = RGV.turningSpeed * progressTime;
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

            times(1) = 0;
            positions(1,:) = startPos;
            eulers(1,:) = startEul;
            movementTypes(1) = RGVMovementType.getRandom();

            counter = 1;
            time = 0;

            while (time <= duration)
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
                    opts = optimset('Display','off');
                    deltaTime = fsolve(@(x) Simulation.DistanceToBondary(RGV.move(time + x, time, positions(counter,:), eulers(counter,:), movementTypes(counter))) - RGV.safeDistanceFromEdge, 0, opts);
                    [newPos, newEuler] = RGV.move(time + deltaTime, time, positions(counter,:), eulers(counter,:), movementTypes(counter));
                    dir = eul2rotm(newEuler)*[1;0;0];
                    if (signedAngle(newPos, dir) > 0)
                        movementTypes(counter+1,:) = RGVMovementType.UTurnLeft;
                    else
                        movementTypes(counter+1,:) = RGVMovementType.UTurnRight;
                    end
                else
                    movementTypes(counter+1,:) = RGVMovementType.getRandom();
                end
                counter = counter + 1;
                time = time + deltaTime;
                times(counter) = time;
                positions(counter,:) = newPos;
                eulers(counter,:) = newEuler;
            end

            this = RGV(times, positions, eulers, movementTypes);
        end
    end
end
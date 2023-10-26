classdef PathSegment
    properties
        pathType (1,1) double
        startTime (1,1) double
        endTime (1,1) double
        startPos (1,3) double
        startRotm (3,3) double
        helperVector1 (1,3) double
        helperScalar1 (1,1) double
    end

    methods
        function this = PathSegment()
            this.pathType = 0;
            this.startTime = 0;
            this.endTime = 0;
            this.startPos = zeros(1,3);
            this.startRotm = eye(3);
            this.helperVector1 = nan(1,3);
            this.helperScalar1 = nan(1);
        end

        function [pos,rotm] = getStateAlongPathAt(this,time)
            switch this.pathType
                case 1
                    speed = this.helperScalar1;
                    dir = this.helperVector1;

                    progressTime = time - this.startTime;
                    pos = this.startPos + speed * progressTime * dir;
                    rotm = this.startRotm;
                case 2
                    turnRate = this.helperScalar1;
                    radiusVectorIn = this.helperVector1;

                    progressTime = time - this.startTime;
                    ang = turnRate*progressTime;
                    angRotm = axang2rotm([0 0 1 ang]);
                    rotm = angRotm*this.startRotm;
                    pos = this.startPos + radiusVectorIn + (angRotm*-radiusVectorIn')';
                otherwise
                    pos = this.startPos;
                    rotm = this.startRotm;
            end
        end

        function this = change(this, startPos, startRotm)
            this.pathType = randi(2);
            this.startPos = startPos;
            this.startRotm = startRotm;
            this.startTime = this.endTime;
            switch this.pathType
                case 1
                    this.endTime = this.startTime + rand(1)*2+2;
                    this.helperVector1 = (this.startRotm * [1;0;0])';
                    this.helperScalar1 = 0.5;
                case 2
                    this.endTime = this.startTime + rand(1)*2+4;
                    radius = rand(1)*5+5;
                    if (rand(1) > 0.5)
                        radius = -radius;
                    end
                    this.helperVector1 = radius*(this.startRotm * [0;1;0])';
                    this.helperScalar1 = 0.5/radius;
                otherwise
                    this.endTime = this.startTime + rand(1)*2+2;
                    this.helperVector1 = nan(1, 3);
                    this.helperScalar1 = nan(1);
            end
        end
    end
end
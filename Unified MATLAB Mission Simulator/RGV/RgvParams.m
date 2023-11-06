classdef RgvParams
    properties
        speed (1,1) double = 0.5;
        turningRadius (1,1) double = 4;
        uTurnRadius (1,1) double = 2;
        waitTimeMin (1,1) double = 90;
        waitTimeMax (1,1) double = 100;
        straightTimeMin (1,1) double = 1;
        straightTimeMax (1,1) double = 2;
        arcTimeMin (1,1) double = 2;
        arcTimeMax (1,1) double = 3;
        waitChanceWeight (1,1) double = 0.1;
        straightChanceWeight (1,1) double = 5;
        arcLeftChanceWeight (1,1) double = 1;
        arcRightChanceWeight (1,1) double = 1;

        % Precalculate
        safeDistanceFromEdge (1,1) double;

        turningSpeed (1,1) double;
        uTurnSpeed (1,1) double;
        uTurnTurnTime (1,1) double;
        uTurnStraightMinTime (1,1) double;
        uTurnTimeMin (1,1) double;
        uTurnTimeMax (1,1) double;

        weightSum (1,1) double;
        waitProbCutoff (1,1) double;
        straightProbCutoff (1,1) double;
        arcLeftCutoff (1,1) double;
    end

    methods
        function this = RgvParams(this)
            this.safeDistanceFromEdge = this.uTurnRadius*2;

            this.turningSpeed =  this.speed / this.turningRadius;
            this.uTurnSpeed =  this.speed / this.uTurnRadius;
            this.uTurnTurnTime = pi / this.uTurnSpeed;
            this.uTurnStraightMinTime = 2 * this.uTurnRadius / this.speed;
            this.uTurnTimeMin = this.uTurnTurnTime + this.uTurnStraightMinTime;
            this.uTurnTimeMax = this.uTurnTurnTime + this.uTurnStraightMinTime + 4;
    
            this.weightSum = this.waitChanceWeight + this.straightChanceWeight + this.arcLeftChanceWeight + this.arcRightChanceWeight;
            this.waitProbCutoff = this.waitChanceWeight / this.weightSum;
            this.straightProbCutoff = (this.waitChanceWeight + this.straightChanceWeight) / this.weightSum;
            this.arcLeftCutoff = (this.waitChanceWeight + this.straightChanceWeight + this.arcLeftChanceWeight) / this.weightSum;
        end
    end
end
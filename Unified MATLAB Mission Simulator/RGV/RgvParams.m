classdef RgvParams
    % A large collection of RGV movement parameters that can be changed
    % each run to alter how the RGV(s) move. This should prevent the need
    % to recompile for small parameter tweaks
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
    end

    properties(Dependent)
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
        function val = get.safeDistanceFromEdge(this)
            val = this.uTurnRadius*2;
        end

        function val = get.turningSpeed(this)
            val = this.speed / this.turningRadius;
        end

        function val = get.uTurnSpeed(this)
            val = this.speed / this.uTurnRadius;
        end

        function val = get.uTurnTurnTime(this)
            val = pi / this.uTurnSpeed;
        end

        function val = get.uTurnStraightMinTime(this)
            val = 2 * this.uTurnRadius / this.speed;
        end

        function val = get.uTurnTimeMin(this)
            val = this.uTurnTurnTime + this.uTurnStraightMinTime;
        end

        function val = get.uTurnTimeMax(this)
            val = this.uTurnTurnTime + this.uTurnStraightMinTime + 4;
        end

        function val = get.weightSum(this)
            val = this.waitChanceWeight + this.straightChanceWeight + this.arcLeftChanceWeight + this.arcRightChanceWeight;
        end

        function val = get.waitProbCutoff(this)
            val = this.waitChanceWeight / this.weightSum;
        end

        function val = get.straightProbCutoff(this)
            val = (this.waitChanceWeight + this.straightChanceWeight) / this.weightSum;
        end

        function val = get.arcLeftCutoff(this)
            val = (this.waitChanceWeight + this.straightChanceWeight + this.arcLeftChanceWeight) / this.weightSum;
        end
    end
end
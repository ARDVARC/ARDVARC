classdef RgvParams
    % A large collection of RGV movement parameters that can be changed
    % each run to alter how the RGV(s) move. This should prevent the need
    % to recompile for small parameter tweaks
    properties
        speed (1,1) double = 1;                 % [m/s] The speed of the RGV whenever it is moving
        turningRadius (1,1) double = 4;         % [m] The radius of the circle that the arcing RGV follows 
        uTurnRadius (1,1) double = 2;           % [m] The radius of the circle that the u-turning RGV follows 
        waitTimeMin (1,1) double = 90;          % [s] The minimum amount of time that a single wait step should last
        waitTimeMax (1,1) double = 100;         % [s] The maximum amount of time that a single wait step should last
        straightTimeMin (1,1) double = 1;       % [s] The minimum amount of time that a single straight step should last
        straightTimeMax (1,1) double = 2;       % [s] The maximum amount of time that a single straight step should last
        arcTimeMin (1,1) double = 2;            % [s] The minimum amount of time that a single arc step should last
        arcTimeMax (1,1) double = 3;            % [s] The maximum amount of time that a single arc step should last
        waitChanceWeight (1,1) double = 0.1;    % How often the RGV should wait relative to other movement types
        straightChanceWeight (1,1) double = 5;  % How often the RGV should go straight relative to other movement types
        arcLeftChanceWeight (1,1) double = 1;   % How often the RGV should arc left relative to other movement types
        arcRightChanceWeight (1,1) double = 1;  % How often the RGV should arc right relative to other movement types
    end

    properties(Dependent)
        safeDistanceFromEdge (1,1) double;      % [m] How far away from the edge is considered "safe" for u-turn determination

        turningSpeed (1,1) double;              % [rad/s] The arc rotation speed
        uTurnSpeed (1,1) double;                % [rad/s] The u-turn rotation speed
        uTurnTurnTime (1,1) double;             % [s] How much time is spent in the turning part of a u-turn
        uTurnStraightMinTime (1,1) double;      % [s] The minimum amount of time that the straight portion of a u-turn step should last
        uTurnTimeMin (1,1) double;              % [s] The minimum amount of time that a single u-turn step should last
        uTurnTimeMax (1,1) double;              % [s] The maximum amount of time that a single u-turn step should last

        weightSum (1,1) double;                 % The sum of all of the RGV movement type weights
        waitProbCutoff (1,1) double;            % The probability below which the RGV will wait
        straightProbCutoff (1,1) double;        % The probability below which the RGV will go straight
        arcLeftCutoff (1,1) double;             % The probability below which the RGV will arc left
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
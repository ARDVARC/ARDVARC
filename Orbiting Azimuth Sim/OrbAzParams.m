classdef OrbAzParams
    % Describes the UAS orbit and sensor details
    properties
        % Seed for rng, set in constructor
        seed (1,1) double;
        % [s] How long the orbit takes
        duration (1,1) double = 60;
        % [m] How far away (ground distance) the UAS orbits the RGV
        orbitDistance (1,1) double = 10;
        % [m] How far up the UAS orbits (AGL)
        height (1,1) double = 10;
        % [m] Standard deviation of orbit height
        heightStd (1,1) double = 0;
        % [deg] Standard deviation of sensor pointing angle error
        angleStdDeg (1,1) double = 2;
        % [Hz] How often the sensor measures
        sampleRate (1,1) double = 5;
        % How many orbits the UAS should do per duration
        orbitCount (1,1) double = 1;
        % Which cost funciton to use
        costFunction (1,1) CostFunctionEnum = CostFunctionEnum.TwoDMoving
    end

    properties(Dependent)
        % [m/s] How fast the UAS should be going
        orbitSpeed (1,1) double
        % [m/s] How fast the UAS should be going (but angular)
        orbitAngularSpeed (1,1) double
        % [rad] Standard deviation of sensor pointing angle error (but radians)
        angleStdRad (1,1) double
        % How many samples will be taken over the entire duration
        sampleCount (1,1) double
    end

    methods
        function this = OrbAzParams(seed)
            arguments(Input)
                seed (1,1) double = randi(intmax);
            end
            arguments(Output)
                this (1,1) OrbAzParams
            end
            this.seed = seed;
        end

        function val = get.orbitSpeed(this)
            val = 2*pi*this.orbitCount*this.orbitDistance/this.duration;
        end
        function this = set.orbitSpeed(this, newVal)
            this.orbitCount = newVal/2/pi/this.orbitDistance*this.duration;
        end
        function val = get.orbitAngularSpeed(this)
            val = this.orbitSpeed / this.orbitDistance;
            if (isnan(val))
                val = 1;
            end
        end
        function this = set.orbitAngularSpeed(this, newVal)
            this.orbitSpeed = newVal * this.orbitDistance;
        end
        function val = get.angleStdRad(this)
            val = deg2rad(this.angleStdDeg);
        end
        function this = set.angleStdRad(this, newVal)
            this.angleStdDeg = rad2deg(newVal);
        end
        function val = get.sampleCount(this)
            val = ceil(this.duration*this.sampleRate);
        end
        function this = set.sampleCount(this, newVal)
            this.sampleRate = newVal/this.duration;
        end
    end
end
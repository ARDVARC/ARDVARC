classdef OrbAzParams
    % Describes the UAS orbit and sensor details
    properties
        seed (1,1) double;                               % Seed for rng, set in constructor
        duration (1,1) double = 30;                      % [s] How long the orbit takes
        orbitDistance (1,1) double = 10;                 % [m] How far away (ground distance) the UAS orbits the RGV
        height (1,1) double = 10;                        % [m] How far up the UAS orbits (AGL)
        heightStd (1,1) double = 0;                      % [m] Standard deviation of orbit height
        angleStdDeg (1,1) double = 3;                    % [deg] Standard deviation of sensor pointing angle error
        sampleRate (1,1) double = 1;                     % [Hz] How often the sensor measures
        orbitCount (1,1) double = 1;                     % How many orbits the UAS should do per duration
        use2DcostFunction (1,1) logical = true;          % Whether to use the 2D cost function to estimate the RGV position. If false, uses 3D cost function
        camParams (1,1) CamParams = CamParams();         % Parameters for the pinhole camera
        usePinholeCameraAsSensor (1,1) logical = true;   % Whether or not to use the pinhole camera sensor model
        showPinholeFrames (1,1) logical = false;         % Whether or not to show each pinhole camera frame
    end

    properties(Dependent)
        orbitSpeed (1,1) double                          % [m/s] How fast the UAS should be going
        orbitAngularSpeed (1,1) double                   % [m/s] How fast the UAS should be going (but angular)
        angleStdRad (1,1) double                         % [rad] Standard deviation of sensor pointing angle error (but radians)
        sampleCount (1,1) double                         % How many samples will be taken over the entire duration
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
classdef OrbAzParams
    % Describes the UAS orbit and sensor details
    properties
        seed (1,1) double = randi(intmax);  % Seed for rng
        duration (1,1) double = 30;        % [s] How long the orbit takes
        orbitDistance (1,1) double = 15;   % [m] How far away (ground distance) the UAS orbits the RGV
        height (1,1) double = 10;          % [m] How far up the UAS orbits (AGL)
        heightStd (1,1) double = 0;        % [m] Standard deviation of orbit height
        angleStdDeg (1,1) double = 3;      % [deg] Standard deviation of sensor pointing angle error
        sampleRate (1,1) double = 1;       % [Hz] How often the sensor measures
        orbitCount (1,1) double = 1;       % How many orbits the UAS should do per duration
    end

    properties(Dependent)
        orbitSpeed (1,1) double            % [m/s] How fast the UAS should be going
        orbitAngularSpeed (1,1) double     % [m/s] How fast the UAS should be going (but angular)
        angleStdRad (1,1) double           % [rad] Standard deviation of sensor pointing angle error (but radians)
        sampleCount (1,1) double           % How many samples will be taken over the entire duration
    end

    methods
        function val = get.orbitSpeed(this)
            val = 2*pi*this.orbitCount*this.orbitDistance/this.duration;
        end
        function this = set.orbitSpeed(this, newVal)
            this.orbitCount = newVal/2/pi/this.orbitDistance*this.duration;
        end
        function val = get.orbitAngularSpeed(this)
            val = this.orbitSpeed / this.orbitDistance;
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
            val = floor(this.duration*this.sampleRate) + 1;
        end
        function this = set.sampleCount(this, newVal)
            this.sampleRate = (newVal-1)/this.duration;
        end
    end
end
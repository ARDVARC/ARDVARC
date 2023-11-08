classdef OrbAzMonteParams
    % Speficies the range of values for a Monte Carlo sim to test
    properties
        minOrbitDistance (1,1) double = 0.001;     % [m] The minimum orbit distance to test
        maxOrbitDistance (1,1) double = 10.001;    % [m] The maximum orbit distance to test
        orbitDistanceGap (1,1) double = 2;   % [m] The size of the gap between each tested orbit distance
        minAngleStdDeg (1,1) double = 1;       % [deg] The minimum pointing angle error (std) to test
        maxAngleStdDeg (1,1) double = 5;       % [deg] The maximum pointing angle error (std) to test
        angleStdGapDeg (1,1) double = 2;       % [deg] The size of the gap between each tested pointing angle error (std)
        sameSeedForAll (1,1) logical = false;  % Whether or not to use the same random seed for each trial
        trialsPerCase (1,1) double = 1000;
    end
end
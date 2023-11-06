classdef RGV
    % Stores known RGV locations at specific times as well as how the RGV
    % starts moving at those times. Generate with makeRgvFromSeed.m.
    properties
        times (:,1) double
        positions (:,3) double
        eulers (:,3) double
        movementTypes (:,1) RgvMovementType
    end

    methods
        function this = RGV(times, positions, eulers, movementTypes)
            this.times = times;
            this.positions = positions;
            this.eulers = eulers;
            this.movementTypes = movementTypes;
        end
    end
end
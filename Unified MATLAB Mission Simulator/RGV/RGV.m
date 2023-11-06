classdef RGV
    properties
        params (1,1) RgvParams
        times (:,1) double
        positions (:,3) double
        eulers (:,3) double
        movementTypes (:,1) RGVMovementType
    end

    methods
        function this = RGV(params, times, positions, eulers, movementTypes)
            this.params = params;
            this.times = times;
            this.positions = positions;
            this.eulers = eulers;
            this.movementTypes = movementTypes;
        end
    end
end
classdef RGV
    % Stores known RGV locations at specific times as well as how the RGV
    % starts moving at those times. Generate with makeRgvFromSeed.m.
    properties
        vec_time (1,:) double
        trix_vec_position_en (2,:) double
        vec_yawAngle (:,1) double
        vec_movementType (1,:) RgvMovementType
    end

    methods
        function this = RGV(vec_time, trix_vec_position_en, vec_yawAngle, vec_movementType)
            this.vec_time = vec_time;
            this.trix_vec_position_en = trix_vec_position_en;
            this.vec_yawAngle = vec_yawAngle;
            this.vec_movementType = vec_movementType;
        end
    end
end
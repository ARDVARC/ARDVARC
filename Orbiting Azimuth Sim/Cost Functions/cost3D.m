function cost = cost3D(vec_rgvPositionEstimate_enu, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu)
    % Given an RGV position estimate, determines how bad of a guess it is
    % by summing the squares of the distances from the guess to each 3D
    % line created by each sensor pointing vector and sample position.
    arguments(Input)
        vec_rgvPositionEstimate_enu (1,3) double      % A guess of where the RGV is
        trix_vec_sensorPointingVec_enu (:,3) double  % A matrix where each row is a sensor pointing vector
        trix_vec_samplePosition_enu (:,3) double     % A matrix where each row is a UAS position
    end
    arguments(Output)
        cost (1,1) double                            % How bad the guess was
    end
    cost = sum(vecnorm(cross(trix_vec_sensorPointingVec_enu,vec_rgvPositionEstimate_enu-trix_vec_samplePosition_enu,2),2,2).^2);
end
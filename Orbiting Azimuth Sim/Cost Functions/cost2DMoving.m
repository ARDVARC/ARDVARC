function cost = cost2DMoving(vec_nodeTimes, vec_vec_rgvPathNode_en, vec_sampleTimes, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, pow)
    % Given a ground-constrained RGV path estimate (a list of different
    % Easting and Northing values at different times), determines how bad
    % of a guess it is by summing the squares of the distances from the
    % guessed path (interpolated linearly) to each 3D line created by each
    % sensor pointing vector and UAS sample position.
    arguments(Input)
        vec_nodeTimes (:,1) double
        vec_vec_rgvPathNode_en (1,:) double
        vec_sampleTimes (:,1) double
        trix_vec_sensorPointingVec_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        pow (1,1) double
    end
    arguments(Output)
        cost (1,1) double                            % How bad the guess was
    end
    
    sampleCount = size(vec_sampleTimes,1);
    trix_vec_rgvPathNode_en = reshape(vec_vec_rgvPathNode_en, 2, [])';
    trix_vec_predictedRgvLocationAtSampleTime_en = interp1(vec_nodeTimes, trix_vec_rgvPathNode_en, vec_sampleTimes);
    cost = sum(vecnorm(cross(trix_vec_sensorPointingVec_enu,[trix_vec_predictedRgvLocationAtSampleTime_en,zeros(sampleCount,1)]-trix_vec_samplePosition_enu,2),2,2).^(pow));
end
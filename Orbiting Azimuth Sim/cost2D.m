function cost = cost2D(trix_vec_rgvPositionEstimate_en, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu)
    N = size(trix_vec_rgvPositionEstimate_en, 1);
    sampleCount = size(trix_vec_rgvPositionEstimate_en, 1);
    trix_vec_rgvPositionEstimate_enu = [trix_vec_rgvPositionEstimate_en,zeros(N,1)];
    
    errorDistances = vecnorm(cross(trix_vec_sensorPointingVec_enu,[trix_vec_rgvPositionEstimate_en,zeros(N,1)]-trix_vec_samplePosition_enu,2),2,2);
    cost = sqrt(sum(errorDistances.^2));
end
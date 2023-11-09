function vec_predictedRgvLocation_enu = getPrediction(trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, params)
    % Predicts the location of the RGV given a number of UAS sample
    % positions and pointing vectors
    arguments(Input)
        trix_vec_sensorPointingVec_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        params (1,1) OrbAzParams
    end
    arguments(Output)
        vec_predictedRgvLocation_enu (1,3) double
    end

    % vec_predictedRgvLocation_enu = fminsearch(@(x) sqrt(sum(vecnorm(cross(trix_vec_sensorPointingVec_enu,x-trix_vec_samplePosition_enu,2),2,2).^2))/params.sampleCount, [0,0,0]);
    vec_predictedRgvLocation_enu = fminsearch(@(x) sqrt(sum(vecnorm(cross(trix_vec_sensorPointingVec_enu,[x,0]-trix_vec_samplePosition_enu,2),2,2).^2))/params.sampleCount, [0,0]);
    vec_predictedRgvLocation_enu = [vec_predictedRgvLocation_enu, 0];
end
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

    if (params.use2DcostFunction)
        vec_predictedRgvLocation_en = fminsearch(@(vec_predictedRgvLocation_en) cost2D(vec_predictedRgvLocation_en,trix_vec_sensorPointingVec_enu,trix_vec_samplePosition_enu), [0,0]);
        vec_predictedRgvLocation_enu = [vec_predictedRgvLocation_en, 0];
    else
        vec_predictedRgvLocation_enu = fminsearch(@(vec_predictedRgvLocation_enu) cost3D(vec_predictedRgvLocation_enu,trix_vec_sensorPointingVec_enu,trix_vec_samplePosition_enu), [0,0,0]);
    end
end
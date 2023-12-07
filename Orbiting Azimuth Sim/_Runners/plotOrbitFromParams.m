function plotOrbitFromParams(params, zoomOnBottom)
    % Creates a 3D plot showing UAS states and pointing angles across a
    % full orbit simulation as specified by params
    arguments(Input)
        params (1,1) OrbAzParams = OrbAzParams();
        zoomOnBottom (1,1) logical = false;
    end

    [trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, vec_predictedRgvLocation_enu] = getDataForParams(params);
    plotOrbitFromData(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, vec_predictedRgvLocation_enu, params, zoomOnBottom)
end
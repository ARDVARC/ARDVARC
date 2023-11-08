function plotOrbitFromParams(params, zoomOnBottom)
    % Creates a 3D plot showing UAS states and pointing angles across a
    % full orbit simulation as specified by params
    arguments(Input)
        params (1,1) OrbAzParams = OrbAzParams();
        zoomOnBottom (1,1) logical = true;
    end

    [trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, bestGuess, ~] = getDataForParams(params);
    plotOrbitFromData(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, bestGuess, params, zoomOnBottom)
end
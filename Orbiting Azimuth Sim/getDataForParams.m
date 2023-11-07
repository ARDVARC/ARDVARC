function [trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, bestGuess, error] = getDataForParams(params)
    % Runs a full orbit simulation based on params and returns important
    % data over the course of the simulation such as UAS states, pointing
    % vectors, estimated position, and error
    arguments(Input)
        params (1,1) OrbAzParams
    end
    arguments(Output)
        trix_vec_samplePosition_enu (:,3) double
        trix_vec_truePointingVec_enu (:,3) double
        trix_vec_sensorPointingVec_enu (:,3) double
        bestGuess (1,3) double
        error (1,1) double
    end
    
    [trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu] = getTrueOrbitValues(params);
    [trix_vec_sensorPointingVec_enu] = getSensorPointingVecs(trix_vec_truePointingVec_enu, params);
    [bestGuess, error] = getPrediction(trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, params);
end
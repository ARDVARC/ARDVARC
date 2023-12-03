function [vec_sampleTimes, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, vec_predictionTimes, trix_vec_predictedRgvLocationAtPredictionTime_enu] = getDataForParams(params)
    % Runs a full orbit simulation based on params and returns important
    % data over the course of the simulation such as UAS states, pointing
    % vectors, estimated position, and error
    arguments(Input)
        params (1,1) OrbAzParams
    end
    arguments(Output)
        vec_sampleTimes (:,1) double
        trix_vec_rgvPosition_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        trix_vec_truePointingVec_enu (:,3) double
        trix_vec_sensorPointingVec_enu (:,3) double
        vec_predictionTimes (:,1) double
        trix_vec_predictedRgvLocationAtPredictionTime_enu (:,3) double
    end
    
    % Determine sample times
    vec_sampleTimes = (0:(1/params.sampleRate):(params.duration-1/params.sampleRate))';

    % Determine the true RGV locations at each sample time
    % trix_vec_rgvPosition_enu = [vec_sampleTimes*0.5, zeros(params.sampleCount,2)];
    trix_vec_rgvPosition_enu = 5*[sind(4*vec_sampleTimes),cosd(4*vec_sampleTimes),zeros(params.sampleCount,1)];

    % Determine the true UAS positions and pointing vectors at each sample
    % time
    angleBetweenSamples = params.orbitAngularSpeed / params.sampleRate;
    sampleAngles = 0:angleBetweenSamples:((params.sampleCount-1)*angleBetweenSamples);
    sampleHeights = randn(1,params.sampleCount)*params.heightStd+params.height;
    trix_vec_samplePosition_enu = [params.orbitDistance*sin(sampleAngles)',params.orbitDistance*cos(sampleAngles)',sampleHeights'];
    trix_vec_longPointingVec_enu = trix_vec_rgvPosition_enu-trix_vec_samplePosition_enu;
    trix_vec_truePointingVec_enu = trix_vec_longPointingVec_enu./vecnorm(trix_vec_longPointingVec_enu,2,2);

    % Shake true pointing vectors to get sensor-like pointing vectors
    [trix_vec_sensorPointingVec_enu] = getSensorPointingVecs(trix_vec_truePointingVec_enu, params);

    % Predict the RGV location based on the samples
    if (params.costFunction == CostFunctionEnum.TwoDMoving)
        vec_predictionTimes = vec_sampleTimes(1):vec_sampleTimes(end);
    else
        vec_predictionTimes = 1;
    end
    trix_vec_predictedRgvLocationAtPredictionTime_enu = getPrediction(vec_sampleTimes, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, vec_predictionTimes, params);
end
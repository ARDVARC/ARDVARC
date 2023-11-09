function [trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu] = getTrueOrbitValues(params)
    % Gets the true sample locations of the UAS based on the orbit
    % described by params
    arguments(Input)
        params (1,1) OrbAzParams
    end
    arguments(Output)
        trix_vec_samplePosition_enu (:,3) double
        trix_vec_truePointingVec_enu (:,3) double
    end

    % Determine the true UAS positions and pointing vectors at each sample
    % time
    angleBetweenSamples = params.orbitAngularSpeed / params.sampleRate;
    sampleAngles = 0:angleBetweenSamples:((params.sampleCount-1)*angleBetweenSamples);
    sampleHeights = randn(1,params.sampleCount)*params.heightStd+params.height;
    trix_vec_samplePosition_enu = [params.orbitDistance*sin(sampleAngles)',params.orbitDistance*cos(sampleAngles)',sampleHeights'];
    trix_vec_truePointingVec_enu = -trix_vec_samplePosition_enu./vecnorm(trix_vec_samplePosition_enu,2,2);
end
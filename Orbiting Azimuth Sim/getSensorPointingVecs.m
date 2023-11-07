function [trix_vec_sensorPointingVec_enu] = getSensorPointingVecs(trix_vec_truePointingVec_enu, params)
    % Shakes the true pointing vectors by an amount specified in
    % params.angleStdDeg to get the sensor's measured pointing vectors
    arguments(Input)
        trix_vec_truePointingVec_enu (:,3) double
        params (1,1) OrbAzParams
    end
    arguments(Output)
        trix_vec_sensorPointingVec_enu (:,3) double
    end

    % Set random number generator seed
    rng(params.seed);
    
    % For each sample, get a specific vector that is orthogonal to the true
    % poining vector. This is stored in a matrix to make the future
    % rotation math easier.
    trix3_vec_specificOrthogonal_enu = zeros(3, 3, params.sampleCount);
    trix3_vec_specificOrthogonal_enu(:,1,:) = [-trix_vec_truePointingVec_enu(:,2) trix_vec_truePointingVec_enu(:,1) zeros(params.sampleCount,1)]';
    % For each sample, generate a random rotation about the true pointing
    % vector.
    trix3_dcm_specificOrthogonal2randomOrthogonal = axang2rotm([trix_vec_truePointingVec_enu rand(params.sampleCount, 1)*2*pi]);
    % For each sample, rotate the specific orthogonal vector by the random
    % rotation to get a random orthogonal vector. This is easy because the
    % specific orthogonal vector is already stored in a matrix.
    trix3_vec_randomOrthogonal_enu = pagemtimes(trix3_dcm_specificOrthogonal2randomOrthogonal, trix3_vec_specificOrthogonal_enu);
    % For each sample, pull the random orthogonal vector out of the matrix.
    trix_vec_randomOrthogonal_enu = reshape(trix3_vec_randomOrthogonal_enu(:,1,:), 3, params.sampleCount)';
    % For each sample, specify the axis and angle of rotation for the
    % estimated pointing vector. The axis is the random orthogonal vector
    % and the angle is random based on the pointing angle standard
    % deviation parameter.
    trix_axangs = [trix_vec_randomOrthogonal_enu randn(params.sampleCount,1)*params.angleStdRad];
    % For each sample, generate the rotation matrix corresponding the the
    % specified axis and angle.
    trix3_dcm_truePointingVec2sensorPointingVec = axang2rotm(trix_axangs);
    % For each sample, put the true pointing vector in a matrix to make the
    % future rotation math easier.
    trix3_vec_truePointingVec_enu = zeros(3, 3, params.sampleCount);
    trix3_vec_truePointingVec_enu(:,1,:) = trix_vec_truePointingVec_enu';
    % For each sample, rotate the true pointing vector by the pointing
    % error rotation to get the sensor pointing vector. This is easy
    % because the specific orthogonal vector is already stored in a matrix.
    trix3_vec_sensorPointingVec_enu = pagemtimes(trix3_dcm_truePointingVec2sensorPointingVec, trix3_vec_truePointingVec_enu);
    % For each sample, pull the true pointing vector out of the matrix.
    trix_vec_sensorPointingVec_enu = reshape(trix3_vec_sensorPointingVec_enu(:,1,:), 3, params.sampleCount)';
end
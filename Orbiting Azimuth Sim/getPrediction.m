function [bestGuess, error] = getPrediction(trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, params)
    % Predicts the location of the RGV given a number of UAS sample
    % positions and pointing vectors
    arguments(Input)
        trix_vec_sensorPointingVec_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        params (1,1) OrbAzParams
    end
    arguments(Output)
        bestGuess (1,3) double
        error (1,1) double
    end

    bestGuess = fminsearch(@(guess) cost2D(guess,trix_vec_sensorPointingVec_enu,trix_vec_samplePosition_enu), [0,0]);
    bestGuess = [bestGuess, 0];
    error = norm(bestGuess);
end
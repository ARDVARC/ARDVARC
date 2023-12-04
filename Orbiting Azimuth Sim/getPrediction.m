function trix_vec_predictedRgvLocationAtPredictionTime_enu = getPrediction(vec_sampleTimes, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, vec_predictionTimes, params)
    % Predicts the location of the RGV given a number of UAS sample
    % positions and pointing vectors
    arguments(Input)
        vec_sampleTimes (:,1) double
        trix_vec_sensorPointingVec_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        vec_predictionTimes (:,1) double
        params (1,1) OrbAzParams
    end
    arguments(Output)
        trix_vec_predictedRgvLocationAtPredictionTime_enu (:,3) double
    end

    switch (params.costFunction)
        case CostFunctionEnum.ThreeD
            assert(size(vec_predictionTimes,1) == 1)
            trix_vec_predictedRgvLocationAtPredictionTime_enu = fminsearch(@(vec_predictedRgvLocation_enu) cost3D(vec_predictedRgvLocation_enu,trix_vec_sensorPointingVec_enu,trix_vec_samplePosition_enu), [100,0,0]);
        case CostFunctionEnum.TwoD
            assert(size(vec_predictionTimes,1) == 1)
            vec_predictedRgvLocation_en = fminsearch(@(vec_predictedRgvLocation_en) cost2D(vec_predictedRgvLocation_en,trix_vec_sensorPointingVec_enu,trix_vec_samplePosition_enu), [100,0]);
            trix_vec_predictedRgvLocationAtPredictionTime_enu = [vec_predictedRgvLocation_en, 0];
        case CostFunctionEnum.TwoDMoving
            vec_coeffs = trix_vec_samplePosition_enu(:,3)./trix_vec_sensorPointingVec_enu(:,3);
            trix_vec_projectedGroundPosition_enu = trix_vec_samplePosition_enu+trix_vec_sensorPointingVec_enu.*-vec_coeffs;
            vec_projectedGroundPosition_en = reshape(trix_vec_projectedGroundPosition_enu(:,1:2),[],1);
            vec_nodeTimes = (vec_sampleTimes(1):5:vec_sampleTimes(end)+5)';
            nodeCount = size(vec_nodeTimes,1);
            trix_vec_timeMatrixPiece = interp1(vec_nodeTimes,diag(ones(1,nodeCount)),vec_sampleTimes);
            trix_vec_timeMatrix = [trix_vec_timeMatrixPiece,zeros(params.sampleCount,nodeCount);zeros(params.sampleCount,nodeCount),trix_vec_timeMatrixPiece];
            trix_vec_invTimeMatrix = pinv(trix_vec_timeMatrix);
            vec_rgvPathNodes_en = trix_vec_invTimeMatrix*vec_projectedGroundPosition_en;
            trix_vec_rgvPathNode_enu = [reshape(vec_rgvPathNodes_en,[],2),zeros(nodeCount,1)];
            trix_vec_predictedRgvLocationAtPredictionTime_enu = interp1(vec_nodeTimes, trix_vec_rgvPathNode_enu, vec_predictionTimes);
    end
end
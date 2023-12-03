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
            vec_nodeTimes = (vec_sampleTimes(1):5:vec_sampleTimes(end)+5)';
            nodeCount = size(vec_nodeTimes,1);
            vec_vec_rgvPathNodeInitialGuess_en = rand([1,nodeCount*2])*20-10;
            % opts = optimset("MaxFunEvals",inf,"MaxIter",inf,'PlotFcns','optimplotfval','TolX',1e-10);
            opts = optimset("MaxFunEvals",1000,"MaxIter",1000,'PlotFcns','optimplotfval');
            vec_vec_rgvPathNode_v1_en = fminsearch(@(vec_vec_rgvPathNode_en) cost2DMoving(vec_nodeTimes, vec_vec_rgvPathNode_en, vec_sampleTimes, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, 4), vec_vec_rgvPathNodeInitialGuess_en, opts);
            opts = optimset("MaxFunEvals",2000,"MaxIter",2000,'PlotFcns','optimplotfval');
            vec_vec_rgvPathNode_v2_en = fminsearch(@(vec_vec_rgvPathNode_en) cost2DMoving(vec_nodeTimes, vec_vec_rgvPathNode_en, vec_sampleTimes, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, 2), vec_vec_rgvPathNode_v1_en, opts);
            opts = optimset("MaxFunEvals",4000,"MaxIter",4000,'PlotFcns','optimplotfval');
            % opts = optimset("MaxFunEvals",inf,"MaxIter",inf,'PlotFcns','optimplotfval','TolX',1e-4);
            vec_vec_rgvPathNode_en = fminsearch(@(vec_vec_rgvPathNode_en) cost2DMoving(vec_nodeTimes, vec_vec_rgvPathNode_en, vec_sampleTimes, trix_vec_sensorPointingVec_enu, trix_vec_samplePosition_enu, 1), vec_vec_rgvPathNode_v2_en, opts);
            trix_vec_rgvPathNode_en = reshape(vec_vec_rgvPathNode_en, 2, [])';
            trix_vec_predictedRgvLocationAtPredictionTime_en = interp1(vec_nodeTimes, trix_vec_rgvPathNode_en, vec_predictionTimes);
            predictionCount = size(vec_predictionTimes,1);
            trix_vec_predictedRgvLocationAtPredictionTime_enu = [trix_vec_predictedRgvLocationAtPredictionTime_en,zeros(predictionCount,1)];
    end
end
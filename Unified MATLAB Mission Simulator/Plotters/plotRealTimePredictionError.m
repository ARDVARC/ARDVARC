function plotRealTimePredictionError()
    [inputFile,inputPath] = uigetfile(".mat","Select Simulation Data File");
    if inputFile == 0
        return
    end
    load([inputPath,inputFile],"truthData","predictorData");
    
    trix3_vec_rgvPositionPredictions_en = permute(predictorData.trix4_splineVars(end,:,:,:),[4,2,3,1]);
    vec_times = truthData.vec_times;
    trix3_vec_rgvPositionPredictions_en = interp1(predictorData.vec_times,trix3_vec_rgvPositionPredictions_en,vec_times,"previous");
    trix3_vec_rgvPositions_en = truthData.trix_vec_rgv1Positions_enu(1:2,:).';
    trix3_vec_rgvPositions_en(:,:,2) = truthData.trix_vec_rgv2Positions_enu(1:2,:).';
    trix3_vec_rgvPositionPredictionErrors_en = trix3_vec_rgvPositionPredictions_en-trix3_vec_rgvPositions_en;
    trix_predictionErrors = permute(vecnorm(trix3_vec_rgvPositionPredictionErrors_en,2,2),[1,3,2]);
    trix_predictionErrors = min(trix_predictionErrors, 6);
    worstError = floor(max(trix_predictionErrors,[],"all"));

    trix_rgvMovementTypes = [truthData.vec_rgv1MovementTypes;truthData.vec_rgv2MovementTypes];

    for rgvNumber = 1:2
        vec_rgvMovementTypes = trix_rgvMovementTypes(rgvNumber,:);
        vec_startWaitTimes = [];
        vec_endWaitTimes = [];
        for i = 2:length(vec_rgvMovementTypes)
            if vec_rgvMovementTypes(i-1) ~= RgvMovementType.Wait && vec_rgvMovementTypes(i) == RgvMovementType.Wait
                vec_startWaitTimes(end+1) = truthData.vec_times(i);
            elseif vec_rgvMovementTypes(i-1) == RgvMovementType.Wait && vec_rgvMovementTypes(i) ~= RgvMovementType.Wait
                vec_endWaitTimes(end+1) = truthData.vec_times(i);
            end
        end
        if vec_rgvMovementTypes(1) == RgvMovementType.Wait
            vec_startWaitTimes = [truthData.vec_times(1),vec_startWaitTimes];
        end
        if vec_rgvMovementTypes(end) == RgvMovementType.Wait
            vec_endWaitTimes = [vec_endWaitTimes,truthData.vec_times(end)];
        end
        ts = zeros(1,length(vec_startWaitTimes)*5-1);
        ys = ts;
        for i = 1:length(vec_startWaitTimes)
            ts(i*5-4:i*5-1) = [vec_startWaitTimes(i),vec_startWaitTimes(i),vec_endWaitTimes(i),vec_endWaitTimes(i)];
            ys(i*5-4:i*5-1) = [0,worstError/2,worstError/2,0]+(rgvNumber-1)*worstError/2;
            if i ~= length(vec_startWaitTimes)
                ts(i*5) = NaN;
                ys(i*5) = NaN;
            end
        end
        waitRegions(rgvNumber) = polyshape(ts,ys);
    end
    
    figure
    grid minor
    hold on
    stairs(vec_times,trix_predictionErrors(:,1),'r');
    stairs(vec_times,trix_predictionErrors(:,2),'b');
    stairs(predictorData.vec_times,predictorData.vec_flightPlanTypes+worstError);
    xline(predictorData.vec_times([false,diff(predictorData.vec_flightPlanTypes)~=0]), 'k:');
    plot(waitRegions(1),"FaceColor","red","EdgeColor","none","FaceAlpha",0.1)
    plot(waitRegions(2),"FaceColor","blue","EdgeColor","none","FaceAlpha",0.1)
    yline([1,2],'k-.')

    xlim([truthData.vec_times(1),truthData.vec_times(end)]);
    ylim([0,worstError+FlightPlanState.JointFocusRGV2+0.25])
end
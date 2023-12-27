function plotPredictedSpeedVsTrue()
    [inputFile,inputPath] = uigetfile(".mat","Select Simulation Data File");
    if inputFile == 0
        return
    end
    load([inputPath,inputFile],"truthData","predictorData","simParams");

    trix_rgvSpeedPredictions = permute(vecnorm(predictorData.trix4_splineVars(end,:,:,:)-predictorData.trix4_splineVars(end-1,:,:,:),2,2)/SimParams.nodeGap,[4,3,2,1]);
    vec_times = truthData.vec_times;
    trix_rgvSpeedPredictions = interp1(predictorData.vec_times,trix_rgvSpeedPredictions,vec_times,"previous");
    trix_rgvSpeedTruths = truthData.vec_rgv1MovementTypes.' ~= RgvMovementType.Wait * simParams.rgvParams.speed;
    trix_rgvSpeedTruths(:,2) = truthData.vec_rgv2MovementTypes.' ~= RgvMovementType.Wait * simParams.rgvParams.speed;
    figure
    hold on
    grid minor
    stairs(vec_times,trix_rgvSpeedPredictions(:,1),'r')
    stairs(vec_times,trix_rgvSpeedPredictions(:,2),'b')
    stairs(vec_times,trix_rgvSpeedTruths(:,1),'r--')
    stairs(vec_times,trix_rgvSpeedTruths(:,2),'b--')
    figure
    hold on
    grid minor
    stairs(vec_times,abs(trix_rgvSpeedPredictions(:,1)-trix_rgvSpeedTruths(:,1)),'r')
    stairs(vec_times,abs(trix_rgvSpeedPredictions(:,2)-trix_rgvSpeedTruths(:,2)),'b')
end
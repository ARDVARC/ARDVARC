function main(simParams)
    % The main function for the entire simulation. Use main.prj to compile
    % this into main_mex which should run a lot faster. Optionally pass in
    % custom SimParams objects to change common simulation parameters.
    arguments(Input)
        simParams (1,1) SimParams = SimParams();
    end

    close all;

    rgv1 = makeRgvFromSeed(simParams.rgvParams, simParams.rgv1Seed, simParams.vec_rgv1startPos_en, simParams.rgv1startYawAngle, simParams.duration, simParams.missionAreaHalfWidth);
    rgv2 = makeRgvFromSeed(simParams.rgvParams, simParams.rgv2Seed, simParams.vec_rgv2startPos_en, simParams.rgv2startYawAngle, simParams.duration, simParams.missionAreaHalfWidth);
    [rawTruthData, sensorData, predictorData] = mainSimulationLoop(simParams, rgv1, rgv2);
    
    vec_sampleTimes = (0:1/simParams.sampleRate:simParams.duration)';
    sampleCount = length(vec_sampleTimes);

    truthData.trix_vec_uasStates = interp1(rawTruthData.vec_times,rawTruthData.trix_vec_uasStates,vec_sampleTimes);
    truthData.vec_times = vec_sampleTimes;

    truthData.trix_vec_rgv1Positions_enu = zeros(3,sampleCount);
    truthData.trix_vec_rgv2Positions_enu = zeros(3,sampleCount);
    truthData.vec_rgv1MovementTypes = zeros(1,sampleCount,"int8");
    truthData.vec_rgv2MovementTypes = zeros(1,sampleCount,"int8");

    for i = 1:sampleCount
        time = truthData.vec_times(i);
        [truthData.trix_vec_rgv1Positions_enu(1:2,i), ~, truthData.vec_rgv1MovementTypes(i)] = getRgvStateAtTime(simParams.rgvParams, rgv1, time);
        [truthData.trix_vec_rgv2Positions_enu(1:2,i), ~, truthData.vec_rgv2MovementTypes(i)] = getRgvStateAtTime(simParams.rgvParams, rgv2, time);
    end

    [file,path] = uiputfile(".mat","Save Simulation Data","ARDVARC_SimData_" + string(datetime('now','TimeZone','local','Format','uuuuMMdd''T''HHmmss')));
    if file == 0
        return;
    end
    save([path,file], "simParams", "truthData", "sensorData", "predictorData");
end
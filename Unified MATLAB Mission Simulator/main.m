function main(simParamsInput)
    % The main function for the entire simulation. Use main.prj to compile
    % this into main_mex which should run a lot faster. Optionally pass in
    % custom SimParams objects to change common simulation parameters.
    arguments(Input)
        simParamsInput (1,1) SimParams = SimParams();
    end

    close all;

    global simParams;
    simParams = simParamsInput;

    rgv1 = RGV.makeFromSeed(simParams.rgv1Seed, simParams.rgv1startPos, simParams.rgv1startEul, simParams.duration);
    rgv2 = RGV.makeFromSeed(simParams.rgv2Seed, simParams.rgv2startPos, simParams.rgv2startEul, simParams.duration);
    [times, trueUasStates, estimatedUasStates, estimatedRgv1Positions, estimatedRgv2Positions] = mainSimulationLoop(rgv1, rgv2, simParams.duration, simParams.uasStartState);
    
    sampleTimes = (0:1/simParams.sampleRate:simParams.duration)';
    sampleCount = length(sampleTimes);

    trueUasStates = interp1(times,trueUasStates,sampleTimes);
    estimatedUasStates = interp1(times,estimatedUasStates,sampleTimes);
    estimatedRgv1Positions = interp1(times,estimatedRgv1Positions,sampleTimes);
    estimatedRgv2Positions = interp1(times,estimatedRgv2Positions,sampleTimes);
    times = sampleTimes;

    trueRgv1Positions = zeros(sampleCount,3);
    trueRgv2Positions = zeros(sampleCount,3);
    rgv1MovementTypes = zeros(sampleCount,1,"int8");
    rgv2MovementTypes = zeros(sampleCount,1,"int8");

    for i = 1:sampleCount
        time = times(i);
        [trueRgv1Positions(i,:), ~, rgv1MovementTypes(i)] = rgv1.getStateAtTime(time);
        [trueRgv2Positions(i,:), ~, rgv2MovementTypes(i)] = rgv2.getStateAtTime(time);
    end

    [idealJointStartIndex, idealJointEndIndex] = plotDistanceBetweenRgvsOverTime(times, trueRgv1Positions, trueRgv2Positions);
    
    [realJointStartIndex, realJointEndIndex] = plotUasDistanceToRgvsOverTime(times, trueRgv1Positions, trueRgv2Positions, trueUasStates(:,1:3));
    
    plotEntireSimulation3D(trueRgv1Positions, rgv1MovementTypes, trueRgv2Positions, rgv2MovementTypes, trueUasStates(:,1:3), idealJointStartIndex, idealJointEndIndex, realJointStartIndex, realJointEndIndex);
end
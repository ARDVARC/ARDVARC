function main_two_rgvs_monte_carlo(runCount, idealJointDuration, sampleRate, duration, rgv1startPos, rgv1startEul, rgv2startPos, rgv2startEul)
    arguments
        runCount = 500;
        idealJointDuration = 20;
        sampleRate = 10;
        duration = 30*60;
        rgv1startPos = [0;-Simulation.missionAreaHalfWidth+RGV.turningRadius;0];
        rgv1startEul = [pi/2;0;0];
        rgv2startPos = [0;Simulation.missionAreaHalfWidth-RGV.turningRadius;0];
        rgv2startEul = [-pi/2;0;0];
    end    
    clc
    close all
    
    times = 0:(1/sampleRate):duration;
    
    bests = zeros(runCount,1);
    for i = 1:runCount
        rgv1 = makeRGVfromSeed_mex(randi(10000), rgv1startPos, rgv1startEul, duration);
        rgv2 = makeRGVfromSeed_mex(randi(10000), rgv2startPos, rgv2startEul, duration);
        [rgv1positions, ~, ~] = getRGVstatesAtTimes_mex(rgv1, times);
        [rgv2positions, ~, ~] = getRGVstatesAtTimes_mex(rgv2, times);
        distanceBetweenRGVs = vecnorm(rgv1positions - rgv2positions, 2, 2);
        movingMaxForJoint = movmax(distanceBetweenRGVs, idealJointDuration*sampleRate);
        bests(i) = min(movingMaxForJoint);
        fprintf("Finished run %i of %i\n", cast(i,"int32"), cast(runCount,"int32"))
    end
    histogram(bests, 0:0.5:(max(bests)+0.5))
    xlabel("Best Sustained Distance [m]")
    ylabel("Frequency")
    grid minor
    title(sprintf("Joint Localization Best Sustained Distances Across %i Simulations", cast(runCount, "int32")))
end
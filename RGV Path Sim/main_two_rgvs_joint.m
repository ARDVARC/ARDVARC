function main_two_rgvs_joint(idealJointDuration, sampleRate, duration, seed1, rgv1startPos, rgv1startEul, seed2, rgv2startPos, rgv2startEul)
    arguments
        idealJointDuration = 20;
        sampleRate = 10;
        duration = 30*60;
        seed1 = randi(10000);
        rgv1startPos = [0;-Simulation.missionAreaHalfWidth+RGV.turningRadius;0];
        rgv1startEul = [pi/2;0;0];
        seed2 = randi(10000);
        rgv2startPos = [0;Simulation.missionAreaHalfWidth-RGV.turningRadius;0];
        rgv2startEul = [-pi/2;0;0];
    end
    clc
    close all
    
    times = 0:(1/sampleRate):duration;
    
    rgv1 = makeRGVfromSeed_mex(seed1, rgv1startPos, rgv1startEul, duration);
    rgv2 = makeRGVfromSeed_mex(seed2, rgv2startPos, rgv2startEul, duration);
    [rgv1positions, ~, rgv1movementTypes] = getRGVstatesAtTimes_mex(rgv1, times);
    [rgv2positions, ~, rgv2movementTypes] = getRGVstatesAtTimes_mex(rgv2, times);
    
    figure
    hold on
    grid minor
    distanceBetweenRGVs = vecnorm(rgv1positions - rgv2positions, 2, 2);
    plot(times, distanceBetweenRGVs, 'k', DisplayName="Distance");
    movingMaxForJoint = movmax(distanceBetweenRGVs, idealJointDuration*sampleRate);
    plot(times,movingMaxForJoint,'r:', DisplayName="Moving Maximum");
    title("Distance Between RGVs vs. Time")
    xlabel("Time [s]")
    ylabel("Distance [m]")
    [minMaxForJoint, minMaxForJointIndex] = min(movingMaxForJoint);
    minMaxForJointStartIndex = max(1, minMaxForJointIndex - idealJointDuration*sampleRate/2);
    minMaxForJointEndIndex = min(length(times), minMaxForJointIndex + idealJointDuration*sampleRate/2);
    fprintf("The best opportunity for joint localization is at t=%.2fs.\nFor %.1fs the max distance between the two RGVs is only %.2fm.\n", ...
        times(minMaxForJointIndex), ...
        idealJointDuration, ...
        minMaxForJoint);
    legend(Location="best")
    
    figure
    hold on
    grid minor
    axis equal
    plot(rgv1positions(:,1),rgv1positions(:,2), DisplayName="RGV1 Path", Color='k')
    plot(rgv2positions(:,1),rgv2positions(:,2), DisplayName="RGV2 Path", Color='b')
    plot(rgv1positions(minMaxForJointStartIndex:minMaxForJointEndIndex,1),rgv1positions(minMaxForJointStartIndex:minMaxForJointEndIndex,2), DisplayName="RGV1 Joint Path", Color='k', Marker='hexagram')
    plot(rgv2positions(minMaxForJointStartIndex:minMaxForJointEndIndex,1),rgv2positions(minMaxForJointStartIndex:minMaxForJointEndIndex,2), DisplayName="RGV2 Joint Path", Color='b', Marker='hexagram')
    title("Entire RGV Path")
    plot([Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth], ...
         [Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth], ...
         DisplayName="Mission Area", ...
         LineStyle=":", ...
         Color="r")
    plot([Simulation.missionAreaHalfWidth - RGV.uTurnRadius*2, -Simulation.missionAreaHalfWidth + RGV.uTurnRadius*2, -Simulation.missionAreaHalfWidth + RGV.uTurnRadius*2, Simulation.missionAreaHalfWidth - RGV.uTurnRadius*2, Simulation.missionAreaHalfWidth - RGV.uTurnRadius*2], ...
         [Simulation.missionAreaHalfWidth - RGV.uTurnRadius*2, Simulation.missionAreaHalfWidth - RGV.uTurnRadius*2, -Simulation.missionAreaHalfWidth + RGV.uTurnRadius*2, -Simulation.missionAreaHalfWidth + RGV.uTurnRadius*2, Simulation.missionAreaHalfWidth - RGV.uTurnRadius*2], ...
         DisplayName="RGV Safe Region", ...
         LineStyle=":", ...
         Color="y")
    stoppedPoints1 = rgv1positions(rgv1movementTypes==RGVMovementType.Wait,:);
    stoppedPoints2 = rgv2positions(rgv2movementTypes==RGVMovementType.Wait,:);
    scatter(stoppedPoints1(:,1),stoppedPoints1(:,2), "k", DisplayName="RGV1 Stop Points")
    scatter(stoppedPoints2(:,1),stoppedPoints2(:,2), "b", DisplayName="RGV2 Stop Points")
    xlabel("X [m]")
    ylabel("Y [m]")
    legend(Location="best")
end
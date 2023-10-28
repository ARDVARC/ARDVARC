clc
clear
close all

idealJointDuration = 20;
plotRate = 10;
duration = 30*60;
seed1 = randi(10000);
seed2 = randi(10000);

[times, rgv1positions, rgv2positions, rgv1movementTypes, rgv2movementTypes] = simulateTwoRGVs(plotRate, duration, seed1, seed2);

figure
hold on
grid minor
distanceBetweenRGVs = vecnorm(rgv1positions - rgv2positions, 2, 2);
plot(times, distanceBetweenRGVs, 'k', DisplayName="Distance");
movingMaxForJoint = movmax(distanceBetweenRGVs, idealJointDuration*plotRate);
plot(times,movingMaxForJoint,'r:', DisplayName="Moving Maximum");
title("Distance Between RGVs vs. Time")
xlabel("Time [s]")
ylabel("Distance [m]")
[minMaxForJoint, minMaxForJointIndex] = min(movingMaxForJoint);
minMaxForJointStartIndex = max(1, minMaxForJointIndex - idealJointDuration*plotRate/2);
minMaxForJointEndIndex = min(length(times), minMaxForJointIndex + idealJointDuration*plotRate/2);
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
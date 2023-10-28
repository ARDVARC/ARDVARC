clc
clear
close all

duration = 30*60;
plotRate = 10;
idealJointDuration = 20;
seed1 = randi(10000);
seed2 = randi(10000);

rgv1 = RGV.makeFromSeed(seed1, [-5 5 0], [0 0 0], duration);
rgv2 = RGV.makeFromSeed(seed2, [5 -5 0], [pi 0 0], duration);

times = 0:(1/plotRate):duration;
rgv1positions = zeros(length(times),3);
rgv2positions = zeros(length(times),3);
rgv1movementTypes = RGVMovementType.empty(0,1);
rgv2movementTypes = RGVMovementType.empty(0,1);
for i = 1:length(times)
    [rgv1positions(i,:), ~, rgv1movementTypes(i)] = rgv1.getStateAtTime(times(i));
    [rgv2positions(i,:), ~, rgv2movementTypes(i)] = rgv2.getStateAtTime(times(i));
end

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
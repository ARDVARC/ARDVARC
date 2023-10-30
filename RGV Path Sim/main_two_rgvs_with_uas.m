clc
clear
close all

idealJointDuration = 20;
plotRate = 10;
duration = 30*60;
seed1 = randi(10000);
seed2 = randi(10000);

[times, rgv1positions, rgv2positions, rgv1movementTypes, rgv2movementTypes, rgv1, rgv2] = simulateTwoRGVs_mex(plotRate, duration, seed1, seed2);

uas = RealisticUAS();
[uasTimes, uasPositions] = uas.simulate(duration, [0;0;10], rgv1, rgv2);
uasPositions(:,3) = -uasPositions(:,3);

figure
hold on
grid minor
distanceBetweenRGVs = vecnorm(rgv1positions - rgv2positions, 2, 2);
plot(times, distanceBetweenRGVs, 'k', DisplayName="Distance");
movingMaxForJointIdeal = movmax(distanceBetweenRGVs, idealJointDuration*plotRate, Endpoints=max(distanceBetweenRGVs));
plot(times,movingMaxForJointIdeal,'r:', DisplayName="Moving Maximum");
title("Distance Between RGVs vs. Time")
xlabel("Time [s]")
ylabel("Distance [m]")
[minMaxForJointIdeal, minMaxForJointIdealIndex] = min(movingMaxForJointIdeal);
minMaxForJointIdealStartIndex = max(1, minMaxForJointIdealIndex - idealJointDuration*plotRate/2);
minMaxForJointIdealEndIndex = min(length(times), minMaxForJointIdealIndex + idealJointDuration*plotRate/2);
fprintf("The ideal opportunity for joint localization is at t=%.2fs.\nFor %.1fs the max distance between the two RGVs is only %.2fm.\n\n", ...
    times(minMaxForJointIdealIndex), ...
    idealJointDuration, ...
    minMaxForJointIdeal);
legend(Location="best")

figure
hold on
grid minor
rgv1PositionsAtUAStimes = getRGVpositionAtTimes_mex(rgv1, uasTimes);
rgv2PositionsAtUAStimes = getRGVpositionAtTimes_mex(rgv2, uasTimes);
UASDistancesToRGV1 = vecnorm(uasPositions - rgv1PositionsAtUAStimes, 2, 2);
UASDistancesToRGV2 = vecnorm(uasPositions - rgv2PositionsAtUAStimes, 2, 2);
plot(uasTimes, UASDistancesToRGV1, 'k', DisplayName="Distance to RGV1");
plot(uasTimes, UASDistancesToRGV2, 'b', DisplayName="Distance to RGV2");
movingMaxForJoint = movmax(max(UASDistancesToRGV1, UASDistancesToRGV2), idealJointDuration*plotRate, Endpoints=max([UASDistancesToRGV1, UASDistancesToRGV2],[],"all"));
plot(uasTimes,movingMaxForJoint,'r:', DisplayName="Moving Maximum");
[minMaxForJoint, minMaxForJointIndex] = min(movingMaxForJoint);
minMaxForJointStartIndex = max(1, minMaxForJointIndex - idealJointDuration*plotRate/2);
minMaxForJointEndIndex = min(length(uasTimes), minMaxForJointIndex + idealJointDuration*plotRate/2);
fprintf("The UAS's best opportunity for joint localization is at t=%.2fs.\nFor %.1fs the max distance to the furthest of the two RGVs is only %.2fm.\n\n", ...
    uasTimes(minMaxForJointIndex), ...
    idealJointDuration, ...
    minMaxForJoint);
title("UAS Distance To RGVs vs. Time")
xlabel("Time [s]")
ylabel("Distance [m]")
legend(Location="best")

figure
plot3(rgv1positions(:,1),rgv1positions(:,2),rgv1positions(:,3), DisplayName="RGV1 Path", Color='k')
hold on
grid minor
axis equal
plot3(rgv2positions(:,1),rgv2positions(:,2),rgv2positions(:,3), DisplayName="RGV2 Path", Color='b')
plot3(rgv1positions(minMaxForJointIdealStartIndex:minMaxForJointIdealEndIndex,1),rgv1positions(minMaxForJointIdealStartIndex:minMaxForJointIdealEndIndex,2),rgv1positions(minMaxForJointIdealStartIndex:minMaxForJointIdealEndIndex,3), DisplayName="RGV1 Ideal Joint Path", Color='k', Marker='hexagram')
plot3(rgv2positions(minMaxForJointIdealStartIndex:minMaxForJointIdealEndIndex,1),rgv2positions(minMaxForJointIdealStartIndex:minMaxForJointIdealEndIndex,2),rgv2positions(minMaxForJointIdealStartIndex:minMaxForJointIdealEndIndex,3), DisplayName="RGV2 Ideal Joint Path", Color='b', Marker='hexagram')
plot3(rgv1PositionsAtUAStimes(minMaxForJointStartIndex:minMaxForJointEndIndex,1),rgv1PositionsAtUAStimes(minMaxForJointStartIndex:minMaxForJointEndIndex,2),rgv1PositionsAtUAStimes(minMaxForJointStartIndex:minMaxForJointEndIndex,3), DisplayName="RGV1 Joint Path", Color='k', Marker='x')
plot3(rgv2PositionsAtUAStimes(minMaxForJointStartIndex:minMaxForJointEndIndex,1),rgv2PositionsAtUAStimes(minMaxForJointStartIndex:minMaxForJointEndIndex,2),rgv2PositionsAtUAStimes(minMaxForJointStartIndex:minMaxForJointEndIndex,3), DisplayName="RGV2 Joint Path", Color='b', Marker='x')
plot3(uasPositions(minMaxForJointStartIndex:minMaxForJointEndIndex,1),uasPositions(minMaxForJointStartIndex:minMaxForJointEndIndex,2),uasPositions(minMaxForJointStartIndex:minMaxForJointEndIndex,3), DisplayName="UAS Joint Path", Color='g', Marker='x')
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
plot3(uasPositions(:,1),uasPositions(:,2),uasPositions(:,3),'g', DisplayName="UAS Path")
xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")
legend(Location="best")
setAxes3DPanAndZoomStyle(zoom(gca),gca,'camera')
clc
clear
close all

duration = 30*60;
plotRate = 10;
seed = randi(10000);

rgv1 = RGV.makeFromSeed(seed, [0 0 0], [0 0 0], duration);

times = 0:(1/plotRate):duration;
positions = zeros(length(times),3);
movementTypes = RGVMovementType.empty(0,1);
for i = 1:length(times)
    [positions(i,:), ~, movementTypes(i)] = rgv1.getStateAtTime(times(i));
end

figure
plot(times, movementTypes, 'k');
title("Movement Type vs. Time")
xlabel("Time [s]")
ylabel("Movement Type")
yticks(0:5)
yticklabels(string([RGVMovementType.Wait, RGVMovementType.Straight, RGVMovementType.ArcLeft, RGVMovementType.ArcRight, RGVMovementType.UTurnLeft, RGVMovementType.UTurnRight]))
grid minor

figure
hold on
grid minor
axis equal
title("XYZ Position Values vs. Time")
x_p = plot(times, positions(:,1), DisplayName="X");
y_p = plot(times, positions(:,2), DisplayName="Y");
z_p = plot(times, positions(:,3), DisplayName="Z");
ma_p = yline(Simulation.missionAreaHalfWidth, "r:", DisplayName="Mission Area");
yline(-Simulation.missionAreaHalfWidth, "r:");
rgvsr_p = yline(Simulation.missionAreaHalfWidth - RGV.turningRadius*2, "y:", DisplayName="RGV Safe Region");
yline(-Simulation.missionAreaHalfWidth + RGV.turningRadius*2, "y:");
legend([x_p, y_p, z_p, ma_p, rgvsr_p], Location="best")
xlabel("Time [s]")
ylabel("Position [m]")
xlim([0 duration])
ylim([-Simulation.missionAreaHalfWidth*1.01 Simulation.missionAreaHalfWidth*1.01])

figure
plot(positions(:,1),positions(:,2), DisplayName="RGV Path", Color='k')
hold on
grid minor
axis equal
title("Entire RGV Path")
plot3([Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth], ...
      [Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, -Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth], ...
      [0, 0, 0, 0, 0], ...
      DisplayName="Mission Area", ...
      LineStyle=":", ...
      Color="r")
plot3([Simulation.missionAreaHalfWidth - RGV.turningRadius*2, -Simulation.missionAreaHalfWidth + RGV.turningRadius*2, -Simulation.missionAreaHalfWidth + RGV.turningRadius*2, Simulation.missionAreaHalfWidth - RGV.turningRadius*2, Simulation.missionAreaHalfWidth - RGV.turningRadius*2], ...
      [Simulation.missionAreaHalfWidth - RGV.turningRadius*2, Simulation.missionAreaHalfWidth - RGV.turningRadius*2, -Simulation.missionAreaHalfWidth + RGV.turningRadius*2, -Simulation.missionAreaHalfWidth + RGV.turningRadius*2, Simulation.missionAreaHalfWidth - RGV.turningRadius*2], ...
      [0, 0, 0, 0, 0], ...
      DisplayName="RGV Safe Region", ...
      LineStyle=":", ...
      Color="y")
xlabel("X [m]")
ylabel("Y [m]")
legend(Location="best")
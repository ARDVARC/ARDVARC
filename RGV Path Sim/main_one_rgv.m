function main_one_rgv(duration, sampleRate, seed, rgvStartPos, rgvStartEul)
    arguments
        duration = 30*60;
        sampleRate = 10;
        seed = randi(10000);
        rgvStartPos = [0;-Simulation.missionAreaHalfWidth+RGV.turningRadius;0];
        rgvStartEul = [pi/2;0;0];
    end
    clc
    close all
    
    coder.extrinsic("yticks")
    coder.extrinsic("yticklabels")
    
    rgv = makeRGVfromSeed_mex(seed, rgvStartPos, rgvStartEul, duration);
    times = 0:(1/sampleRate):duration;
    [rgvPositions, ~, rgvMovementTypes] = getRGVstatesAtTimes_mex(rgv, times);
    
    figure
    plot(times, rgvMovementTypes, 'k');
    title("Movement Type vs. Time")
    xlabel("Time [s]")
    ylabel("Movement Type")
    yticks(0:5)
    yticklabels({"Wait", "Straight", "ArcLeft", "ArcRight", "UTurnLeft", "UTurnRight"})
    grid minor
    
    figure
    hold on
    grid minor
    title("XYZ Position Values vs. Time")
    x_p = plot(times, rgvPositions(:,1), DisplayName="X");
    y_p = plot(times, rgvPositions(:,2), DisplayName="Y");
    z_p = plot(times, rgvPositions(:,3), DisplayName="Z");
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
    plot(rgvPositions(:,1),rgvPositions(:,2), DisplayName="RGV Path", Color='k')
    hold on
    grid minor
    axis equal
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
    stoppedPoints = rgvPositions(rgvMovementTypes==RGVMovementType.Wait,:);
    scatter(stoppedPoints(:,1),stoppedPoints(:,2), "r", "filled", DisplayName="Stop Points")
    xlabel("X [m]")
    ylabel("Y [m]")
    legend(Location="best")
end
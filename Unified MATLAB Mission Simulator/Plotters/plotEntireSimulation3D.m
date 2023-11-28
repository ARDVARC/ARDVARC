function plotEntireSimulation3D(rgv1Positions, rgv1movementTypes, rgv2Positions, rgv2movementTypes, uasPositions, idealJointStartIndex, idealJointEndIndex, realJointStartIndex, realJointEndIndex)
    % Plots the entire mission simulation, including both RGV and UAS paths
    arguments(Input)
        rgv1Positions (:,3) double
        rgv1movementTypes (:,1) RgvMovementType
        rgv2Positions (:,3) double
        rgv2movementTypes (:,1) RgvMovementType
        uasPositions (:,3) double
        idealJointStartIndex (1,1) double        % Index for the time when joint localization would ideally start (based on RGV proximity)
        idealJointEndIndex (1,1) double          % Index for the time when joint localization would ideally end (based on RGV proximity)
        realJointStartIndex (1,1) double         % Index for the best time when joint localization could actually start (based on UAS proximity)
        realJointEndIndex (1,1) double           % Index for the best time when joint localization could actually end (based on UAS proximity)
    end
    
    coder.extrinsic("util.plotting.slideify")

    global simParams;

    uasPositions(:,3) = -uasPositions(:,3);

    figure
    plot3(rgv1Positions(:,1),rgv1Positions(:,2),rgv1Positions(:,3), DisplayName="RGV1 Path", Color='#ffbb00')
    hold on
    grid minor
    axis equal
    plot3(rgv2Positions(:,1),rgv2Positions(:,2),rgv2Positions(:,3), DisplayName="RGV2 Path", Color='#0077ff')
    plot3([simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth], ...
         [simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth], ...
         [0.1, 0.1, 0.1, 0.1, 0.1],...
         DisplayName="Mission Area", ...
         Color="#d13928")
    stoppedPoints1 = rgv1Positions(rgv1movementTypes==RgvMovementType.Wait,:);
    stoppedPoints2 = rgv2Positions(rgv2movementTypes==RgvMovementType.Wait,:);
    scatter(stoppedPoints1(:,1),stoppedPoints1(:,2), MarkerEdgeColor="#ffbb00", DisplayName="RGV1 Stop Points")
    scatter(stoppedPoints2(:,1),stoppedPoints2(:,2), MarkerEdgeColor="#0077ff", DisplayName="RGV2 Stop Points")
    plot3(uasPositions(:,1),uasPositions(:,2),uasPositions(:,3), Color='#1aff00', DisplayName="UAS Path")
    title("Mission Simulation")
    xlabel("X [m]")
    ylabel("Y [m]")
    zlabel("Z [m]")
    legend(Location="southoutside")
    util.plotting.slideify()
end
function plotEntireSimulation3D(rgv1Positions, rgv1movementTypes, rgv2Positions, rgv2movementTypes, uasPositions, idealJointStartIndex, idealJointEndIndex, realJointStartIndex, realJointEndIndex)
    % Plots the entire mission simulation, including both RGV and UAS paths
    arguments(Input)
        rgv1Positions (:,3) double
        rgv1movementTypes (:,1) RGVMovementType
        rgv2Positions (:,3) double
        rgv2movementTypes (:,1) RGVMovementType
        uasPositions (:,3) double
        idealJointStartIndex (1,1) double        % Index for the time when joint localization would ideally start (based on RGV proximity)
        idealJointEndIndex (1,1) double          % Index for the time when joint localization would ideally end (based on RGV proximity)
        realJointStartIndex (1,1) double         % Index for the best time when joint localization could actually start (based on UAS proximity)
        realJointEndIndex (1,1) double           % Index for the best time when joint localization could actually end (based on UAS proximity)
    end

    global simParams;

    uasPositions(:,3) = -uasPositions(:,3);

    figure
    plot3(rgv1Positions(:,1),rgv1Positions(:,2),rgv1Positions(:,3), DisplayName="RGV1 Path", Color='k')
    hold on
    grid minor
    axis equal
    plot3(rgv2Positions(:,1),rgv2Positions(:,2),rgv2Positions(:,3), DisplayName="RGV2 Path", Color='b')
    plot3(rgv1Positions(idealJointStartIndex:idealJointEndIndex,1),rgv1Positions(idealJointStartIndex:idealJointEndIndex,2),rgv1Positions(idealJointStartIndex:idealJointEndIndex,3), DisplayName="RGV1 Ideal Joint Path", Color='k', Marker='hexagram')
    plot3(rgv2Positions(idealJointStartIndex:idealJointEndIndex,1),rgv2Positions(idealJointStartIndex:idealJointEndIndex,2),rgv2Positions(idealJointStartIndex:idealJointEndIndex,3), DisplayName="RGV2 Ideal Joint Path", Color='b', Marker='hexagram')
    plot3(rgv1Positions(realJointStartIndex:realJointEndIndex,1),rgv1Positions(realJointStartIndex:realJointEndIndex,2),rgv1Positions(realJointStartIndex:realJointEndIndex,3), DisplayName="RGV1 Joint Path", Color='k', Marker='x')
    plot3(rgv2Positions(realJointStartIndex:realJointEndIndex,1),rgv2Positions(realJointStartIndex:realJointEndIndex,2),rgv2Positions(realJointStartIndex:realJointEndIndex,3), DisplayName="RGV2 Joint Path", Color='b', Marker='x')
    plot3(uasPositions(realJointStartIndex:realJointEndIndex,1),uasPositions(realJointStartIndex:realJointEndIndex,2),uasPositions(realJointStartIndex:realJointEndIndex,3), DisplayName="UAS Joint Path", Color='g', Marker='x')
    plot([simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth], ...
         [simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, -simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth], ...
         DisplayName="Mission Area", ...
         LineStyle=":", ...
         Color="r")
    plot([simParams.missionAreaHalfWidth - simParams.rgvParams.uTurnRadius*2, -simParams.missionAreaHalfWidth + simParams.rgvParams.uTurnRadius*2, -simParams.missionAreaHalfWidth + simParams.rgvParams.uTurnRadius*2, simParams.missionAreaHalfWidth - simParams.rgvParams.uTurnRadius*2, simParams.missionAreaHalfWidth - simParams.rgvParams.uTurnRadius*2], ...
         [simParams.missionAreaHalfWidth - simParams.rgvParams.uTurnRadius*2, simParams.missionAreaHalfWidth - simParams.rgvParams.uTurnRadius*2, -simParams.missionAreaHalfWidth + simParams.rgvParams.uTurnRadius*2, -simParams.missionAreaHalfWidth + simParams.rgvParams.uTurnRadius*2, simParams.missionAreaHalfWidth - simParams.rgvParams.uTurnRadius*2], ...
         DisplayName="RGV Safe Region", ...
         LineStyle=":", ...
         Color="y")
    stoppedPoints1 = rgv1Positions(rgv1movementTypes==RGVMovementType.Wait,:);
    stoppedPoints2 = rgv2Positions(rgv2movementTypes==RGVMovementType.Wait,:);
    scatter(stoppedPoints1(:,1),stoppedPoints1(:,2), "k", DisplayName="RGV1 Stop Points")
    scatter(stoppedPoints2(:,1),stoppedPoints2(:,2), "b", DisplayName="RGV2 Stop Points")
    plot3(uasPositions(:,1),uasPositions(:,2),uasPositions(:,3),'g', DisplayName="UAS Path")
    title("Entire RGV Path")
    xlabel("X [m]")
    ylabel("Y [m]")
    zlabel("Z [m]")
    legend(Location="best")
end
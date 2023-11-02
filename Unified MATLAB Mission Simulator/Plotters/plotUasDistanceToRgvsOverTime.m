function [realJointStartIndex, realJointEndIndex] = plotUasDistanceToRgvsOverTime(times, rgv1Positions, rgv2Positions, uasPositions)
    arguments(Input)
        times (1,:) double
        rgv1Positions (:,3) double
        rgv2Positions (:,3) double
        uasPositions (:,3) double
    end
    arguments(Output)
        realJointStartIndex (:,1) double
        realJointEndIndex (:,1) double
    end

    global simParams;

    UASDistancesToRGV1 = vecnorm(uasPositions - rgv1Positions, 2, 2);
    UASDistancesToRGV2 = vecnorm(uasPositions - rgv2Positions, 2, 2);
    movingMaxForJoint = movmax(max(UASDistancesToRGV1, UASDistancesToRGV2), simParams.idealJointDuration*simParams.sampleRate, Endpoints=max([UASDistancesToRGV1, UASDistancesToRGV2],[],"all"));
    [minMaxForJoint, minMaxForJointIndex] = min(movingMaxForJoint);
    realJointStartIndex = max(1, minMaxForJointIndex - simParams.idealJointDuration*simParams.sampleRate/2);
    realJointEndIndex = min(length(times), minMaxForJointIndex + simParams.idealJointDuration*simParams.sampleRate/2);

    fprintf("The UAS's best opportunity for joint localization is at t=%.2fs.\nFor %.1fs the max distance to the furthest of the two RGVs is only %.2fm.\n\n", ...
        times(minMaxForJointIndex), ...
        simParams.idealJointDuration, ...
        minMaxForJoint);

    figure
    hold on
    grid minor
    plot(times, UASDistancesToRGV1, 'k', DisplayName="Distance to RGV1");
    plot(times, UASDistancesToRGV2, 'b', DisplayName="Distance to RGV2");
    plot(times,movingMaxForJoint,'r:', DisplayName="Moving Maximum");
    title("UAS Distance To RGVs vs. Time")
    xlabel("Time [s]")
    ylabel("Distance [m]")
    legend(Location="best")
end
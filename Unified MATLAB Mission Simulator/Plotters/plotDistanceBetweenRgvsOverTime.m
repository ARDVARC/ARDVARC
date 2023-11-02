function [idealJointStartIndex, idealJointEndIndex] = plotDistanceBetweenRgvsOverTime(times, rgv1Positions, rgv2Positions)
    arguments(Input)
        times (1,:) double
        rgv1Positions (:,3) double
        rgv2Positions (:,3) double
    end
    arguments(Output)
        idealJointStartIndex (1,1) double
        idealJointEndIndex (1,1) double
    end

    global simParams;

    distanceBetweenRGVs = vecnorm(rgv1Positions - rgv2Positions, 2, 2);
    movingMaxForJointIdeal = movmax(distanceBetweenRGVs, simParams.idealJointDuration*simParams.sampleRate, Endpoints=max(distanceBetweenRGVs));
    [minMaxForJointIdeal, minMaxForJointIdealIndex] = min(movingMaxForJointIdeal);
    idealJointStartIndex = max(1, minMaxForJointIdealIndex - simParams.idealJointDuration*simParams.sampleRate/2);
    idealJointEndIndex = min(length(times), minMaxForJointIdealIndex + simParams.idealJointDuration*simParams.sampleRate/2);
    
    fprintf("The ideal opportunity for joint localization is at t=%.2fs.\nFor %.1fs the max distance between the two RGVs is only %.2fm.\n\n", ...
        times(minMaxForJointIdealIndex), ...
        simParams.idealJointDuration, ...
        minMaxForJointIdeal);

    figure
    hold on
    grid minor
    plot(times, distanceBetweenRGVs, 'k', DisplayName="Distance");
    plot(times, movingMaxForJointIdeal,'r:', DisplayName="Moving Maximum");
    title("Distance Between RGVs vs. Time")
    xlabel("Time [s]")
    ylabel("Distance [m]")
    legend(Location="best")
end
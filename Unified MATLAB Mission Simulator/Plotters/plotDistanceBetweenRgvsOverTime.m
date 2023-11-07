function [idealJointStartIndex, idealJointEndIndex] = plotDistanceBetweenRgvsOverTime(times, rgv1Positions, rgv2Positions)
    % Plots the distance between the two RGVs over the duration of the
    % mission
    arguments(Input)
        times (1,:) double
        rgv1Positions (:,3) double
        rgv2Positions (:,3) double
    end
    arguments(Output)
        idealJointStartIndex (1,1) double  % Index for the time when joint localization would ideally start (based on RGV proximity)
        idealJointEndIndex (1,1) double    % Index for the time when joint localization would ideally end (based on RGV proximity)
    end

    global simParams;

    distanceBetweenRgvs = vecnorm(rgv1Positions - rgv2Positions, 2, 2);
    movingMaxForJointIdeal = movmax(distanceBetweenRgvs, simParams.idealJointDuration*simParams.sampleRate, Endpoints=max(distanceBetweenRgvs));
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
    plot(times, distanceBetweenRgvs, 'k', DisplayName="Distance");
    plot(times, movingMaxForJointIdeal,'r:', DisplayName="Moving Maximum");
    title("Distance Between RGVs vs. Time")
    xlabel("Time [s]")
    ylabel("Distance [m]")
    legend(Location="best")
end
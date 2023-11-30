function plotDistanceBetweenRgvsOverTime
    % Plots the distance between the two RGVs over the duration of the
    % mission

    [file, path] = uigetfile(".mat","Choose Simulation Data File");
    load([path,file]);

    distanceBetweenRgvs = vecnorm(rgv1Positions - rgv2Positions);
    movingMaxForJointIdeal = movmax(distanceBetweenRgvs, simParams.idealJointDuration*simParams.sampleRate, Endpoints=max(distanceBetweenRgvs));
    [minMaxForJointIdeal, minMaxForJointIdealIndex] = min(movingMaxForJointIdeal);
    
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
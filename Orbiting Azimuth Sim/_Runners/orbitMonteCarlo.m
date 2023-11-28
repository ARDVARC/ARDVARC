function orbitMonteCarlo(monteParams, params)
    % Runs the orbit sim for a range of different orbital parameters
    % (specifically varying orbital distance and pointing angle error) and
    % plots how estimate error varies as a function of those two parameters
    arguments(Input)
        monteParams (1,1) OrbAzMonteParams = OrbAzMonteParams();
        params (1,1) OrbAzParams = OrbAzParams();
    end

    close all
    
    % Determine the range and number of orbital distances and angle errors
    % to test
    orbitDistances = monteParams.minOrbitDistance:monteParams.orbitDistanceGap:monteParams.maxOrbitDistance;
    angleStdDegs = monteParams.minAngleStdDeg:monteParams.angleStdGapDeg:monteParams.maxAngleStdDeg;
    orbitDistanceCount = length(orbitDistances);
    angleStdDegsCount = length(angleStdDegs);

    % Preallocate space for all of the different predicted RGV locations
    % - Each column is a different predicted RGV location
    % - The column index says which angle error is being tested
    % - The page index says which orbit distance is being tested
    % - The hyperpage(?) index says which trial the data is for (each
    %       scenario has many trials)
    trix4_vec_predictedRgvLocation_enu = zeros(3,angleStdDegsCount,orbitDistanceCount,monteParams.trialsPerCase);
    
    % Do all the trials, varying simulation parameters as needed
    tic
    for i = 1:angleStdDegsCount
        params.angleStdDeg = angleStdDegs(i);
        for j = 1:orbitDistanceCount
            for k = 1:monteParams.trialsPerCase
                params.orbitDistance = orbitDistances(j);
                [~, ~, ~, trix4_vec_predictedRgvLocation_enu(:,i,j,k)] = getDataForParams(params);
                if (~monteParams.sameSeedForAll)
                    params.seed = randi(intmax);
                end
            end
        end
    end
    toc

    % Note: The RGV is at [0,0,0] so the predicted RGV location is also the
    % error vector

    % Calculate standard deviations across the trials for each scenario for 
    % each axis
    trix_sigma_x = permute(std(trix4_vec_predictedRgvLocation_enu(1,:,:,:),0,4), [2,3,1]);
    trix_sigma_y = permute(std(trix4_vec_predictedRgvLocation_enu(2,:,:,:),0,4), [2,3,1]);
    trix_sigma_z = permute(std(trix4_vec_predictedRgvLocation_enu(3,:,:,:),0,4), [2,3,1]);
    % Calculate the average of the magnitudes of the error vectors across
    % the trials for each scenario
    trix_meanErrorMagnitude = permute(mean(vecnorm(trix4_vec_predictedRgvLocation_enu,2,1),4), [2,3,1]);
    % Calculate the average error vector across the trials for each
    % scenario (aka the bias vector)
    trix3_vec_bias_enu = mean(trix4_vec_predictedRgvLocation_enu,4);
    % Calculate the magnitude of the bias vector for each scenario 
    trix_biasMagnitude = permute(vecnorm(trix3_vec_bias_enu,2,1), [2,3,1]);
    % Calculate the 2DRMS across the trials for each scenario using the
    % equation gove here:
    % https://www.gnss.ca/app_notes/APN-029_GPS_Position_Accuracy_Measures_Application_Note.html
    trix_twoDRMS = 2*sqrt(trix_sigma_x.^2+trix_sigma_y.^2);
    
    % Plot 2DRMS
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,trix_twoDRMS(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    yline(2, 'k', DisplayName="Coarse Localization Target")
    yline(1, 'r', DisplayName="Fine Localization Target")
    xlabel("Orbit Distance [m]")
    ylabel("Estimate 2DRMS [m]")
    title("RGV Position Estimate Error 2DRMS")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix_twoDRMS, [], "all")])
    legend(Location="best")
    set(gcf,"Color","#f0b1ad")
    
    % Plot mean error magnitude
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,trix_meanErrorMagnitude(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("Mean Estimate Error [m]")
    title("RGV Position Estimate Mean Error")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylimupper = max(trix_meanErrorMagnitude, [], "all");
    ylim([0 ylimupper])
    legend(Location="best")
    set(gcf,"Color","#f0d7ad")
    
    % Plot bias magnitude
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,trix_biasMagnitude(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("Bias Magnitude [m]")
    title("RGV Position Estimate Bias Magnitude")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylimupper = max(trix_biasMagnitude, [], "all");
    ylim([0 ylimupper])
    legend(Location="best")
    set(gcf,"Color","#edf0ad")
    
    % Plot each sigma
    figure
    hold on
    grid minor
    sgtitle("RGV Position Estimate Error Standard Deviations by Axis")
    set(gcf,"Color","#cef0ad")
    subplot(3,1,1)
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,trix_sigma_x(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("sigma_x [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix_sigma_x, [], "all")])
    legend(Location="best")
    subplot(3,1,2)
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,trix_sigma_y(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("sigma_y [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix_sigma_y, [], "all")])
    subplot(3,1,3)
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,trix_sigma_z(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("sigma_z [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    if (trix_sigma_z ~= 0)
        ylim([0 max(trix_sigma_z, [], "all")])
    end
    
    % Plot bias by axis
    figure
    hold on
    grid minor
    sgtitle("RGV Position Estimate Bias by Axis")
    set(gcf,"Color","#adf0bd")
    subplot(3,1,1)
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix3_vec_bias_enu(1,i,:),[2,3,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("X Bias [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([min(trix3_vec_bias_enu(1,:,:), [], "all") max(trix3_vec_bias_enu(1,:,:), [], "all")])
    legend(Location="best")
    subplot(3,1,2)
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix3_vec_bias_enu(2,i,:),[2,3,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("Y Bias [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([min(trix3_vec_bias_enu(2,:,:), [], "all") max(trix3_vec_bias_enu(2,:,:), [], "all")])
    subplot(3,1,3)
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix3_vec_bias_enu(3,i,:),[2,3,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("Z Bias [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    if (trix3_vec_bias_enu(3,:,:) ~= 0)
        ylim([min(trix3_vec_bias_enu(3,:,:), [], "all") max(trix3_vec_bias_enu(3,:,:), [], "all")])
    end
end
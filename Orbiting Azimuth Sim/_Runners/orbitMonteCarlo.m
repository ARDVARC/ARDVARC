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
    trix_sigma_x = std(trix4_vec_predictedRgvLocation_enu(1,:,:,:),0,4);
    trix_sigma_y = std(trix4_vec_predictedRgvLocation_enu(2,:,:,:),0,4);
    trix_sigma_z = std(trix4_vec_predictedRgvLocation_enu(3,:,:,:),0,4);
    % Calculate the average of the magnitudes of the error vectors across
    % the trials for each scenario
    trix_meanErrorMagnitude = mean(vecnorm(trix4_vec_predictedRgvLocation_enu,2,1),4);
    % Calculate the average error vector across the trials for each
    % scenario (aka the bias vector)
    trix3_vec_bias_enu = mean(trix4_vec_predictedRgvLocation_enu,4);
    % Calculate the magnitude of the bias vector for each scenario 
    trix_biasMagnitude = vecnorm(trix3_vec_bias_enu,2,1);
    % Calculate the 2DRMS across the trials for each scenario using the
    % equation gove here:
    % https://www.gnss.ca/app_notes/APN-029_GPS_Position_Accuracy_Measures_Application_Note.html
    trix_twoDRMS = 2*sqrt(trix_sigma_x.^2+trix_sigma_y.^2);
    
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix_twoDRMS(1,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
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
    
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix_meanErrorMagnitude(1,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("Mean Estimate Error [m]")
    title("RGV Position Estimate Mean Error")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylimlower = min(trix_meanErrorMagnitude, [], "all");
    ylimupper = max(trix_meanErrorMagnitude, [], "all");
    ylim([ylimlower ylimupper])
    legend(Location="best")
    set(gcf,"Color","#f0d7ad")
    
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix_biasMagnitude(1,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("Bias Magnitude [m]")
    title("RGV Position Estimate Bias Magnitude")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylimlower = min(trix_biasMagnitude, [], "all");
    ylimupper = max(trix_biasMagnitude, [], "all");
    ylim([ylimlower ylimupper])
    legend(Location="best")
    set(gcf,"Color","#edf0ad")
    
    figure
    hold on
    grid minor
    sgtitle("RGV Position Estimate Error Standard Deviations by Axis")
    set(gcf,"Color","#cef0ad")
    subplot(3,1,1)
    hold on
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix_sigma_x(1,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("sigma_x [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix_sigma_x, [], "all")])
    legend(Location="best")
    subplot(3,1,2)
    hold on
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix_sigma_y(1,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("sigma_y [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix_sigma_y, [], "all")])
    subplot(3,1,3)
    hold on
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix_sigma_z(1,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("sigma_z [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix_sigma_z, [], "all")])
    
    figure
    hold on
    grid minor
    sgtitle("RGV Position Estimate Bias by Axis")
    set(gcf,"Color","#adf0bd")
    subplot(3,1,1)
    hold on
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix3_vec_bias_enu(1,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("X Bias [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix3_vec_bias_enu(1,:,:), [], "all")])
    legend(Location="best")
    subplot(3,1,2)
    hold on
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix3_vec_bias_enu(2,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    ylabel("Y Bias [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix3_vec_bias_enu(2,:,:), [], "all")])
    subplot(3,1,3)
    hold on
    for i = 1:angleStdDegsCount
        plot(orbitDistances,permute(trix3_vec_bias_enu(3,i,:),[3,2,1]),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    xlabel("Orbit Distance [m]")
    ylabel("Z Bias [m]")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(trix3_vec_bias_enu(3,:,:), [], "all")])
end
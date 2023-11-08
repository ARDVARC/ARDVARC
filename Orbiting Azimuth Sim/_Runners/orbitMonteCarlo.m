function orbitMonteCarlo(monteParams, params)
    % Runs the orbit sim for a range of different orbital parameters
    % (specifically varying orbital distance and pointing angle error) and
    % plots how estimate error varies as a function of those two parameters
    arguments(Input)
        monteParams (1,1) OrbAzMonteParams = OrbAzMonteParams();
        params (1,1) OrbAzParams = OrbAzParams();
    end

    close all
    
    orbitDistances = monteParams.minOrbitDistance:monteParams.orbitDistanceGap:monteParams.maxOrbitDistance;
    angleStdDegs = monteParams.minAngleStdDeg:monteParams.angleStdGapDeg:monteParams.maxAngleStdDeg;
    orbitDistanceCount = length(orbitDistances);
    angleStdDegsCount = length(angleStdDegs);
    errors = zeros(angleStdDegsCount,orbitDistanceCount,monteParams.trialsPerCase);
    
    tic
    for i = 1:angleStdDegsCount
        params.angleStdDeg = angleStdDegs(i);
        for j = 1:orbitDistanceCount
            for k = 1:monteParams.trialsPerCase
                params.orbitDistance = orbitDistances(j);
                if (~monteParams.sameSeedForAll)
                    params.seed = randi(intmax);
                end
                [~, ~, ~, ~, errors(i,j,k)] = getDataForParams(params);
            end
        end
    end
    toc

    errorStds = std(errors, 0, 3);
    errorMeans = mean(errors, 3);
    
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,errorMeans(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    yline(2, 'k', DisplayName="Coarse Localization Target")
    yline(1, 'r', DisplayName="Fine Localization Target")
    xlabel("Orbit Distance [m]")
    ylabel("Estimate Error [m]")
    title("RGV Position Estimate Error for Different Orbital Ground Distances")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(errorMeans, [], "all")])
    legend(Location="best")
    
    figure
    hold on
    grid minor
    for i = 1:angleStdDegsCount
        plot(orbitDistances,errorStds(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    yline(2, 'k', DisplayName="Coarse Localization Target")
    yline(1, 'r', DisplayName="Fine Localization Target")
    xlabel("Orbit Distance [m]")
    ylabel("Estimate Error Standard Deviation [m]")
    title("RGV Position Estimate Error Standard Deviation for Different Orbital Ground Distances")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(errorStds, [], "all")])
    legend(Location="best")
end
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
    errors = zeros(angleStdDegsCount,orbitDistanceCount);
    
    tic
    for i = 1:angleStdDegsCount
        params.angleStdDeg = angleStdDegs(i);
        for j = 1:orbitDistanceCount
            params.orbitDistance = orbitDistances(j);
            if (~monteParams.sameSeedForAll)
                params.seed = randi(10000);
            end
            [~, ~, ~, ~, errors(i,j)] = getDataForParams(params);
        end
    end
    toc
    
    figure
    hold on
    grid minor
    axis equal
    for i = 1:angleStdDegsCount
        scatter(orbitDistances,errors(i,:),Marker=".",DisplayName=sprintf("%.2f Degree STD", angleStdDegs(i)))
    end
    yline(2, 'k:', DisplayName="Coarse Localization Target")
    yline(1, 'r:', DisplayName="Fine Localization Target")
    xlabel("Orbit Distance [m]")
    ylabel("Estimate Error [m]")
    title("RGV Position Estimate Error for Different Orbital Ground Distances")
    xlim([monteParams.minOrbitDistance monteParams.maxOrbitDistance])
    ylim([0 max(errors, [], "all")])
    legend(Location="best")
end
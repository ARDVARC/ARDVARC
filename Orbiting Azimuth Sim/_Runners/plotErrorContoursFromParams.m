function plotErrorContoursFromParams(params)
    arguments(Input)
        params (1,1) OrbAzParams = OrbAzParams();
    end

    [trix_vec_samplePosition_enu, ~, trix_vec_sensorPointingVec_enu, bestGuess, ~] = getDataForParams(params);

    figure
    hold on
    grid minor
    axis equal
    fcontour(@(x,y) cost2D([x,y],trix_vec_sensorPointingVec_enu,trix_vec_samplePosition_enu))
    scatter(bestGuess(1), bestGuess(2), 10, "red", "filled")
end
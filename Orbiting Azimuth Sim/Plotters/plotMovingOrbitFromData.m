function plotMovingOrbitFromData(vec_sampleTimes, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, vec_predictionTimes, trix_vec_predictedRgvLocationAtPredictionTime_enu, params)
    arguments(Input)
        vec_sampleTimes (:,1) double
        trix_vec_rgvPosition_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        trix_vec_truePointingVec_enu (:,3) double
        trix_vec_sensorPointingVec_enu (:,3) double
        vec_predictionTimes (:,1) double
        trix_vec_predictedRgvLocationAtPredictionTime_enu (:,3) double
        params (1,1) OrbAzParams
    end
    
    close all

    figure(1)
    plots(1) = scatter3(trix_vec_samplePosition_enu(:,1),trix_vec_samplePosition_enu(:,2),trix_vec_samplePosition_enu(:,3),DisplayName="True UAS Locations");
    hold on
    grid minor
    axis equal
    plots(2) = plot3(trix_vec_rgvPosition_enu(:,1),trix_vec_rgvPosition_enu(:,2),trix_vec_rgvPosition_enu(:,3),'bsquare-',DisplayName="True RGV Location(s)");
    plots(3) = plot3(trix_vec_predictedRgvLocationAtPredictionTime_enu(:,1),trix_vec_predictedRgvLocationAtPredictionTime_enu(:,2),trix_vec_predictedRgvLocationAtPredictionTime_enu(:,3),'msquare-.',DisplayName="Guessed RGV Location(s)");

    plots(4) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, 1, 'g', "True Poining Vectors");
    plots(5) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu*2*params.height, 1, 'g--', "True Poining Line");
    plots(6) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu, 1, 'r', "Sensor Poining Vectors");
    plots(7) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu*2*params.height, 1, 'r--', "Sensor Poining Line");
    legend(plots, Location="best")
end
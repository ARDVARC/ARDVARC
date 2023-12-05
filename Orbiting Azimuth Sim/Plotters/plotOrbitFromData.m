function [uasPlot, rgvTruePlot, rgvSplinePlot, timeVaryingPlots] = plotOrbitFromData(ax, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, trix_vec_predictedRgvLocationAtPredictionTime_enu, params)
    arguments(Input)
        ax (1,1) matlab.ui.control.UIAxes
        trix_vec_rgvPosition_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        trix_vec_truePointingVec_enu (:,3) double
        trix_vec_sensorPointingVec_enu (:,3) double
        trix_vec_predictedRgvLocationAtPredictionTime_enu (:,3) double
        params (1,1) OrbAzParams
    end
    
    uasPlot = scatter3(ax,trix_vec_samplePosition_enu(:,1),trix_vec_samplePosition_enu(:,2),trix_vec_samplePosition_enu(:,3),DisplayName="True UAS Locations");
    grid(ax,"on")
    hold(ax,"on")
    rgvTruePlot = plot3(ax,trix_vec_rgvPosition_enu(:,1),trix_vec_rgvPosition_enu(:,2),trix_vec_rgvPosition_enu(:,3),'bsquare-',DisplayName="True RGV Location(s)");
    rgvSplinePlot = plot3(ax,trix_vec_predictedRgvLocationAtPredictionTime_enu(:,1),trix_vec_predictedRgvLocationAtPredictionTime_enu(:,2),trix_vec_predictedRgvLocationAtPredictionTime_enu(:,3),'msquare-.',DisplayName="Guessed RGV Location(s)");

    timeVaryingPlots = matlab.graphics.chart.primitive.Line.empty(params.sampleCount,4,0);
    timeVaryingPlots(:,1,1) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, ax, 'g', "True Poining Vectors");
    timeVaryingPlots(:,2,1) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu*5*params.height, ax, 'g--', "True Poining Line");
    timeVaryingPlots(:,3,1) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu, ax, 'r', "Sensor Poining Vectors");
    timeVaryingPlots(:,4,1) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu*5*params.height, ax, 'r--', "Sensor Poining Line");
    legend(ax,[rgvTruePlot, rgvSplinePlot, timeVaryingPlots(1,:)],Location="best")
end
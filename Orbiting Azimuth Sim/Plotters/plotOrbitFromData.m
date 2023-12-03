function plotOrbitFromData(ax, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, trix_vec_predictedRgvLocationAtPredictionTime_enu, params)
    arguments(Input)
        ax (1,1) matlab.ui.control.UIAxes
        trix_vec_rgvPosition_enu (:,3) double
        trix_vec_samplePosition_enu (:,3) double
        trix_vec_truePointingVec_enu (:,3) double
        trix_vec_sensorPointingVec_enu (:,3) double
        trix_vec_predictedRgvLocationAtPredictionTime_enu (:,3) double
        params (1,1) OrbAzParams
    end
    
    plots(1) = scatter3(ax,trix_vec_samplePosition_enu(:,1),trix_vec_samplePosition_enu(:,2),trix_vec_samplePosition_enu(:,3),DisplayName="True UAS Locations");
    grid(ax,"on")
    hold(ax,"on")
    % axis vis3d
    % setAxes3DPanAndZoomStyle(zoom(gca),gca,'camera')
    plots(2) = plot3(ax,trix_vec_rgvPosition_enu(:,1),trix_vec_rgvPosition_enu(:,2),trix_vec_rgvPosition_enu(:,3),'bsquare-',DisplayName="True RGV Location(s)");
    plots(3) = plot3(ax,trix_vec_predictedRgvLocationAtPredictionTime_enu(:,1),trix_vec_predictedRgvLocationAtPredictionTime_enu(:,2),trix_vec_predictedRgvLocationAtPredictionTime_enu(:,3),'msquare-.',DisplayName="Guessed RGV Location(s)");

    plots(4) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, ax, 'g', "True Poining Vectors");
    plots(5) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu*5*params.height, ax, 'g--', "True Poining Line");
    plots(6) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu, ax, 'r', "Sensor Poining Vectors");
    plots(7) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu*5*params.height, ax, 'r--', "Sensor Poining Line");
    % legend(plots,Location="best")
    % legend(ax)
end
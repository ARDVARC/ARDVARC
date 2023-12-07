function plotOrbitFromData(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, bestGuess, params, zoomOnBottom)
    arguments(Input)
        trix_vec_samplePosition_enu (:,3) double
        trix_vec_truePointingVec_enu (:,3) double
        trix_vec_sensorPointingVec_enu (:,3) double
        bestGuess (1,3) double
        params (1,1) OrbAzParams
        zoomOnBottom (1,1) logical = false;
    end
    
    close all

    figure(1)
    plots(1) = scatter3(trix_vec_samplePosition_enu(:,1),trix_vec_samplePosition_enu(:,2),trix_vec_samplePosition_enu(:,3),DisplayName="True UAS Locations");
    hold on
    grid minor
    axis equal
    % axis vis3d
    % setAxes3DPanAndZoomStyle(zoom(gca),gca,'camera')
    plots(2) = scatter3(0,0,0,'g','filled','square',DisplayName="True RGV Location");
    plots(3) = scatter3(bestGuess(1),bestGuess(2),bestGuess(3),'r','filled','square',DisplayName="Guessed RGV Location");
    if (zoomOnBottom)
        xlim([-params.orbitDistance/4 params.orbitDistance/4])
        ylim([-params.orbitDistance/4 params.orbitDistance/4])
        zlim([-params.orbitDistance/4 params.orbitDistance/2])
    else
        xlim([-params.orbitDistance params.orbitDistance])
        ylim([-params.orbitDistance params.orbitDistance])
        zlim([0 params.height + 3*params.heightStd])
    end

    plots(4) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, 1, 'g', "True Poining Vectors");
    plots(5) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu*2*params.height, 1, 'g--', "True Poining Line");
    plots(6) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu, 1, 'r', "Sensor Poining Vectors");
    plots(7) = plot3DVectorsComingFrom(trix_vec_samplePosition_enu, trix_vec_sensorPointingVec_enu*2*params.height, 1, 'r--', "Sensor Poining Line");
    legend(plots, Location="best")
end
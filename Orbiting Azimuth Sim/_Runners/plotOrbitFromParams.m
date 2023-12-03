function plotOrbitFromParams(params)
    % Creates a 3D plot showing UAS states and pointing angles across a
    % full orbit simulation as specified by params
    arguments(Input)
        params (1,1) OrbAzParams = OrbAzParams();
    end

    close all force
    
    [vec_sampleTimes, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, vec_predictionTimes, trix_vec_predictedRgvLocationAtPredictionTime_enu] = getDataForParams(params);
    
    fig = uifigure;
    g = uigridlayout(fig);
    g.RowHeight = {'1x','fit'};
    g.ColumnWidth = {'1x'};
    ax = uiaxes(g);
    grid(ax,"minor")
    view(ax,3)
    sld = uislider(g,"Limits",[0 params.duration]);
    sld.ValueChangingFcn = @(~,event) sliderCallback(ax, event.Value, vec_sampleTimes, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, vec_predictionTimes, trix_vec_predictedRgvLocationAtPredictionTime_enu, params);
end
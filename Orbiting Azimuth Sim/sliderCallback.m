function sliderCallback(ax, time, vec_sampleTimes, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, vec_predictionTimes, trix_vec_predictedRgvLocationAtPredictionTime_enu, uasPlot, rgvTruePlot, rgvSplinePlot, timeVaryingPlots, params)
    indexOfSampleTimeBefore = find(vec_sampleTimes<=time-2,1,"last");
    if (isempty(indexOfSampleTimeBefore))
        indexOfSampleTimeBefore = 1;
    end
    indexOfSampleTimeAfter = find(vec_sampleTimes>=time+2,1,"first");
    if (isempty(indexOfSampleTimeAfter))
        indexOfSampleTimeAfter = length(vec_sampleTimes);
    end
    indexOfPredictionTimeBefore = find(vec_predictionTimes<=time-2,1,"last");
    if (isempty(indexOfPredictionTimeBefore))
        indexOfPredictionTimeBefore = 1;
    end
    indexOfPredictionTimeAfter = find(vec_predictionTimes>=time+2,1,"first");
    if (isempty(indexOfPredictionTimeAfter))
        indexOfPredictionTimeAfter = length(vec_predictionTimes);
    end

    trix_vec_rgvPosition_enu = trix_vec_rgvPosition_enu(indexOfSampleTimeBefore:indexOfSampleTimeAfter,:);
    trix_vec_samplePosition_enu = trix_vec_samplePosition_enu(indexOfSampleTimeBefore:indexOfSampleTimeAfter,:);
    trix_vec_predictedRgvLocationAtPredictionTime_enu = trix_vec_predictedRgvLocationAtPredictionTime_enu(indexOfPredictionTimeBefore:indexOfPredictionTimeAfter,:);

    set(uasPlot, "XData", trix_vec_samplePosition_enu(:,1), "YData", trix_vec_samplePosition_enu(:,2), "ZData", trix_vec_samplePosition_enu(:,3))
    set(rgvTruePlot, "XData", trix_vec_rgvPosition_enu(:,1), "YData", trix_vec_rgvPosition_enu(:,2), "ZData", trix_vec_rgvPosition_enu(:,3))
    set(rgvSplinePlot, "XData", trix_vec_predictedRgvLocationAtPredictionTime_enu(:,1), "YData", trix_vec_predictedRgvLocationAtPredictionTime_enu(:,2), "ZData", trix_vec_predictedRgvLocationAtPredictionTime_enu(:,3))
    
    for i = 1:params.sampleCount
        if (indexOfSampleTimeBefore <= i && indexOfSampleTimeAfter >= i)
            visibility = "on";
        else
            visibility = "off";
        end
        timeVaryingPlots(i,1).Visible = visibility;
        timeVaryingPlots(i,2).Visible = visibility;
        timeVaryingPlots(i,3).Visible = visibility;
        timeVaryingPlots(i,4).Visible = visibility;
    end
    drawnow
end
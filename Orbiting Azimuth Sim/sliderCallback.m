function sliderCallback(ax, time, vec_sampleTimes, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, vec_predictionTimes, trix_vec_predictedRgvLocationAtPredictionTime_enu, params)
    persistent timeOfLastCall

    if (isempty(timeOfLastCall))
        timeOfLastCall = datetime("now");
    end

    if ((datetime("now") - timeOfLastCall) < duration(0,0,0,500))
        return
    else
        timeOfLastCall = datetime("now");
    end


    cla(ax)
    trix_vec_allPoints_enu = [trix_vec_rgvPosition_enu;trix_vec_samplePosition_enu;trix_vec_predictedRgvLocationAtPredictionTime_enu];
    axis(ax,"equal")
    xlim(ax,[min(trix_vec_allPoints_enu(:,1)), max(trix_vec_allPoints_enu(:,1))])
    ylim(ax,[min(trix_vec_allPoints_enu(:,2)), max(trix_vec_allPoints_enu(:,2))])
    zlim(ax,[min(trix_vec_allPoints_enu(:,3)), max(trix_vec_allPoints_enu(:,3))])

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
    trix_vec_truePointingVec_enu = trix_vec_truePointingVec_enu(indexOfSampleTimeBefore:indexOfSampleTimeAfter,:);
    trix_vec_sensorPointingVec_enu = trix_vec_sensorPointingVec_enu(indexOfSampleTimeBefore:indexOfSampleTimeAfter,:);
    trix_vec_predictedRgvLocationAtPredictionTime_enu = trix_vec_predictedRgvLocationAtPredictionTime_enu(indexOfPredictionTimeBefore:indexOfPredictionTimeAfter,:);
    plotOrbitFromData(ax, trix_vec_rgvPosition_enu, trix_vec_samplePosition_enu, trix_vec_truePointingVec_enu, trix_vec_sensorPointingVec_enu, trix_vec_predictedRgvLocationAtPredictionTime_enu, params)
end
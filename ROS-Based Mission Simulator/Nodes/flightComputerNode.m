function flightComputerNode()
    if ~isCoreRunning(rosdevice('localhost'))
        warning("Start simulation node first")
        return;
    end
    rosshutdown;
    rosinit;
    rgvPositionPredictorNode = ros.Node("rgvPositionPredictor","localhost");
    disp("Running RGV Position Predictor node")

    rgv1PositionPredictionsPublisher = ros.Publisher(rgvPositionPredictorNode,"rgv1_position_predictions","ardvarc/Point","DataFormat","struct");
    rgv2PositionPredictionsPublisher = ros.Publisher(rgvPositionPredictorNode,"rgv2_position_predictions","ardvarc/Point","DataFormat","struct");
    measuredUasPoseSubscriber = ros.Subscriber(rgvPositionPredictorNode,"uas_pose_measurements","ardvarc/Pose","DataFormat","struct");
    rgv1PositionMeasurementsSubscriber = ros.Subscriber(rgvPositionPredictorNode,"rgv1_position_measurements","ardvarc/Point","DataFormat","struct");
    rgv2PositionMeasurementsSubscriber = ros.Subscriber(rgvPositionPredictorNode,"rgv2_position_measurements","ardvarc/Point","DataFormat","struct");

    rgv1PositionPredictionMsg = rosmessage(rgv1PositionPredictionsPublisher);
    rgv2PositionPredictionMsg = rosmessage(rgv2PositionPredictionsPublisher);
    measuredUasPoseMsg = rosmessage(measuredUasPoseSubscriber);
    rgv1PositionMeasurementMsg = rosmessage(rgv1PositionMeasurementsSubscriber);
    rgv2PositionMeasurementMsg = rosmessage(rgv2PositionMeasurementsSubscriber);

    r = rosrate(1);
    while true
        waitfor(r);
        if ~isempty(measuredUasPoseSubscriber.LatestMessage)
            measuredUasPoseMsg = measuredUasPoseSubscriber.LatestMessage;
        end
        if ~isempty(rgv1PositionMeasurementsSubscriber.LatestMessage)
            rgv1PositionMeasurementMsg = rgv1PositionMeasurementsSubscriber.LatestMessage;
        end
        if ~isempty(rgv2PositionMeasurementsSubscriber.LatestMessage)
            rgv2PositionMeasurementMsg = rgv2PositionMeasurementsSubscriber.LatestMessage;
        end

        rgv1PositionPredictionsPublisher.send(rgv1PositionPredictionMsg);
        rgv2PositionPredictionsPublisher.send(rgv2PositionPredictionMsg);

        rosShowDetails(measuredUasPoseMsg)
        rosShowDetails(rgv1PositionMeasurementMsg)
        rosShowDetails(rgv2PositionMeasurementMsg)
    end
end
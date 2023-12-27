function bluetoothSensorCallback(src,msg,bluetoothSensor)
    timeNow = rostime("now");
    switch src.TopicName
        case "/sim/true_rgv1_positions"
            if timeNow.Sec > bluetoothSensor.rgv1TimeOfNextPublish.Sec || (timeNow.Sec == bluetoothSensor.rgv1TimeOfNextPublish.Sec && timeNow.Nsec >= bluetoothSensor.rgv1TimeOfNextPublish.Nsec) 
                bluetoothSensor.rgv1TimeOfNextPublish = bluetoothSensor.rgv1TimeOfNextPublish + bluetoothSensor.timeBetweenPublishes;
                disp(rosShowDetails(msg))
                bluetoothSensor.rgv1PositionMeasurementPublisher.send(msg);
            end
        case "/sim/true_rgv2_positions"
            if timeNow.Sec > bluetoothSensor.rgv2TimeOfNextPublish.Sec || (timeNow.Sec == bluetoothSensor.rgv2TimeOfNextPublish.Sec && timeNow.Nsec >= bluetoothSensor.rgv2TimeOfNextPublish.Nsec) 
                bluetoothSensor.rgv2TimeOfNextPublish = bluetoothSensor.rgv2TimeOfNextPublish + bluetoothSensor.timeBetweenPublishes;
                disp(rosShowDetails(msg))
                bluetoothSensor.rgv2PositionMeasurementPublisher.send(msg);
            end
    end
end
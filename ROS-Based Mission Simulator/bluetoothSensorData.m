classdef bluetoothSensorData < handle
    properties
        rgv1PositionMeasurementPublisher (1,1) ros.Publisher = rospublisher("rgv1_position_measurements","ardvarc/Point","DataFormat","struct");
        rgv2PositionMeasurementPublisher (1,1) ros.Publisher = rospublisher("rgv2_position_measurements","ardvarc/Point","DataFormat","struct");
        rgv1TimeOfNextPublish (1,1) ros.msg.Time;
        rgv2TimeOfNextPublish (1,1) ros.msg.Time;
        timeBetweenPublishes (1,1) ros.msg.Duration = ros.msg.Duration(0,0.5e9);
    end
    methods
        function obj = bluetoothSensorData()
            obj.rgv1TimeOfNextPublish = rostime("now");
            obj.rgv2TimeOfNextPublish = obj.rgv1TimeOfNextPublish + obj.timeBetweenPublishes/2;
        end
    end
end
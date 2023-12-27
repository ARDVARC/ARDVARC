classdef uasStateSensorData < handle
    properties
        uasPoseMeasurementPublisher (1,1) ros.Publisher = rospublisher("uas_pose_measurements","ardvarc/Pose","DataFormat","struct")
        timeOfNextPublish (1,1) ros.msg.Time = rostime("now");
        timeBetweenPublishes (1,1) ros.msg.Duration = ros.msg.Duration(0,0.5e9);
    end
end
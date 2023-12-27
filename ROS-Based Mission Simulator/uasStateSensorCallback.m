function uasStateSensorCallback(~,msg,uasStateSensor)
    timeNow = rostime("now");
    if timeNow.Sec > uasStateSensor.timeOfNextPublish.Sec || (timeNow.Sec == uasStateSensor.timeOfNextPublish.Sec && timeNow.Nsec >= uasStateSensor.timeOfNextPublish.Nsec) 
        uasStateSensor.timeOfNextPublish = uasStateSensor.timeOfNextPublish + uasStateSensor.timeBetweenPublishes;
        disp(rosShowDetails(msg));
        uasStateSensor.uasPoseMeasurementPublisher.send(msg);
    end
end
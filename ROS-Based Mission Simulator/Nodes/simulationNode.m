function simulationNode()
    rosshutdown;
    rosinit;
    disp("Running Simulation node")

    % Create all simulation publishers
    trueUasPosePublisher = rospublisher("/sim/true_uas_poses","ardvarc/Pose","DataFormat","struct");
    trueRgv1PositionPublisher = rospublisher("/sim/true_rgv1_positions","ardvarc/Point","DataFormat","struct");
    trueRgv2PositionPublisher = rospublisher("/sim/true_rgv2_positions","ardvarc/Point","DataFormat","struct");
    
    % Make sure topics you subscribe to exist
    rospublisher("flight_plans","ardvarc/FlightPlan");

    % Create subscribers
    flightPlanSubscriber = rossubscriber("flight_plans","ardvarc/FlightPlan","DataFormat","struct");
    trueUasPoseSubscriber = rossubscriber("/sim/true_uas_poses","ardvarc/Pose","DataFormat","struct");
    uasStateSensor = uasStateSensorData;
    trueUasPoseSubscriber.NewMessageFcn = {@uasStateSensorCallback, uasStateSensor};
    trueRgv1PositionSubscriber = rossubscriber("/sim/true_rgv1_positions","ardvarc/Point","DataFormat","struct");
    trueRgv2PositionSubscriber = rossubscriber("/sim/true_rgv2_positions","ardvarc/Point","DataFormat","struct");
    bluetoothSensor = bluetoothSensorData;
    trueRgv1PositionSubscriber.NewMessageFcn = {@bluetoothSensorCallback, bluetoothSensor};
    trueRgv2PositionSubscriber.NewMessageFcn = {@bluetoothSensorCallback, bluetoothSensor};

    % Initialize physics simulation
    uasState = [1;1;1];
    rgv1State = [1;0;0];
    rgv2State = [0;1;0];
    flightPlanMsg = rosmessage(flightPlanSubscriber);
    uasPoseMsg = rosmessage(trueUasPosePublisher);
    rgv1PositionMsg = rosmessage(trueRgv1PositionPublisher);
    rgv2PositionMsg = rosmessage(trueRgv2PositionPublisher);

    r = rosrate(100);
    while true
        % Check for new flight plans
        if ~isempty(flightPlanSubscriber.LatestMessage)
            flightPlanMsg = flightPlanSubscriber.LatestMessage;
        end
        % Simulate physics
        uasState = 0.9 * uasState + 0.1 * flightPlanMsg.GoToCenter.Point_;
        % Update RGV positions
        rgv1State = rgv1State + rand(3,1)-0.5;
        rgv2State = rgv2State + rand(3,1)-0.5;
        % Prepare new messages
        uasPoseMsg.Position.Point_ = uasState;
        rgv1PositionMsg.Point_ = rgv1State;
        rgv2PositionMsg.Point_ = rgv2State;
        % Wait to publish new truth data
        waitfor(r);
        % Publish new messages
        trueUasPosePublisher.send(uasPoseMsg);
        trueRgv1PositionPublisher.send(rgv1PositionMsg);
        trueRgv2PositionPublisher.send(rgv2PositionMsg);
    end
end
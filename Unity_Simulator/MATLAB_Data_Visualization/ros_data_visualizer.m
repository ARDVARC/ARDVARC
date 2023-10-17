clc
clear
close all

if (ros.internal.Global.isNodeActive == 0)
    rosinit("127.0.0.1",11311)
end

uas_state_sub = rossubscriber("uas_state");
rgv1_state_sub = rossubscriber("rgv1_state");
rgv2_state_sub = rossubscriber("rgv2_state");
figure
plot3(0,0,0);
uas_path = animatedline(Color='r',MaximumNumPoints=1000);
rgv1_path = animatedline(Color='b',MaximumNumPoints=1000);
rgv2_path = animatedline(Color='g',MaximumNumPoints=1000);

hold on
grid minor
axis equal
xlim([-25 25])
ylim([-25 25])
zlim([0 15])
while (true)
    [msg,status,~] = receive(uas_state_sub,1);
    if (status == 1)
        addpoints(uas_path, msg.Pos.X, msg.Pos.Z, msg.Pos.Y);
        drawnow
    end
    [msg,status,~] = receive(rgv1_state_sub,1);
    if (status == 1)
        addpoints(rgv1_path, msg.Pos.X, msg.Pos.Z, msg.Pos.Y);
        drawnow
    end
    [msg,status,~] = receive(rgv2_state_sub,1);
    if (status == 1)
        addpoints(rgv2_path, msg.Pos.X, msg.Pos.Z, msg.Pos.Y);
        drawnow
    end
end

rosshutdown
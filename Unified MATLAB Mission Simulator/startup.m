% Add relevant paths
addpath("util")
addpath("RGV")
addpath("UAS")
addpath("UAS/Control")
addpath("UAS/Sensors")
addpath("Physics")
addpath("Plotters")
addpath("Simulation")
addpath("_Runners")
disp("Paths added by startup.m")

% Add example sim params to workspace
sp_1_2 = SimParams();
sp_1_2.rgv1Seed = 1;
sp_1_2.rgv2Seed = 2;
sp_quick = SimParams();
sp_quick.rgv1Seed = 2;
sp_quick.rgv2Seed = 3;
sp_quick.duration = 30;
sp_quick.vec_uasStartState = zeros(12,1);
sp_quick.vec_uasStartState(3) = -sp_quick.targetUasHeight;
sp_quick.vec_rgv1startPos_en = sp_quick.vec_rgv1startPos_en * 0.1;
sp_quick.rgv1startYawAngle = pi;
sp_quick.vec_rgv2startPos_en = sp_quick.vec_rgv2startPos_en * 0.1;
sp_quick.rgv2startYawAngle = 0;
disp("Example sim params added by startup.m")
% Add relevant paths
addpath("util")
addpath("RGV")
addpath("UAS/Control")
addpath("UAS/Sensors")
addpath("Physics")
addpath("Plotters")
addpath("Simulation")
addpath("_Runners")
disp("Paths added by startup.m")

% Add example sim params to workspace
global simParams;
sp_1_2 = SimParams();
sp_1_2.rgv1Seed = 1;
sp_1_2.rgv2Seed = 2;
simParams = sp_1_2;
sp_quick = SimParams();
sp_quick.rgv1Seed = 2;
sp_quick.rgv2Seed = 3;
sp_quick.duration = 30;
sp_quick.uasStartState = zeros(12,1);
sp_quick.uasStartState(3) = -sp_quick.targetUasHeight;
sp_quick.rgv1startPos = sp_quick.rgv1startPos * 0.1;
sp_quick.rgv1startEul = [pi;0;0];
sp_quick.rgv2startPos = sp_quick.rgv2startPos * 0.1;
sp_quick.rgv2startEul = [0;0;0];
disp("Example sim params added by startup.m")
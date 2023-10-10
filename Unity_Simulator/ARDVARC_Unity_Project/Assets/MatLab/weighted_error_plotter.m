clc
clear
close all

wind_multiplier = 20;

[file, path] = uigetfile("*.csv","Select Data CSV","..\DataDump\DroneWeights");
data = readmatrix(path + "" + file);

data = data(data(:,1)>20,:);

Times = data(:,1);
WeightedVerticalPositionError = data(:,2);
WeightedVerticalVelocityError = data(:,3);
Z = data(:,4);
WeightedRollRateError = data(:,5);
WeightedRollAngleError = data(:,6);
WeightedHorizontalPositionError = data(:,7);
WeightedHorizontalVelocityError = data(:,8);
L = data(:,9);
WeightedPitchRateError = data(:,10);
WeightedPitchAngleError = data(:,11);
WeightedForwardPositionError = data(:,12);
WeightedForwardVelocityError = data(:,13);
M = data(:,14); % M is pitch up
WeightedYawRateError = data(:,15);
WeightedYawAngleError = data(:,16);
N = data(:,17);
pitch = data(:,18);
roll = data(:,19);
yaw = data(:,20);
u = data(:,21);
v = data(:,22);
w = data(:,23);
pitchRate = data(:,24);
yawRate = data(:,25);
rollRate = data(:,26);
wind = data(:,27) * wind_multiplier;
rgv1moving = (data(:,28) - 0.5) * 50;
gravityError = data(:,29);
WeightedForwardPositionErrorForZ = data(:,30);

figure
hold on
grid minor
% smartplot(Times, WeightedVerticalPositionError);
% smartplot(Times, WeightedVerticalVelocityError);
% smartplot(Times, gravityError);
% smartplot(Times, WeightedForwardPositionErrorForZ);
% smartplot(Times, Z);
% smartplot(Times, w);
% smartplot(Times, WeightedRollRateError);
% smartplot(Times, WeightedRollAngleError);
% smartplot(Times, WeightedHorizontalPositionError);
% smartplot(Times, WeightedHorizontalVelocityError);
% smartplot(Times, L);
% smartplot(Times, roll);
% smartplot(Times, v);
% smartplot(Times, rollRate);
smartplot(Times, WeightedPitchRateError);
smartplot(Times, WeightedPitchAngleError);
smartplot(Times, WeightedForwardPositionError);
smartplot(Times, WeightedForwardVelocityError);
smartplot(Times, M);
% smartplot(Times, pitch);
% smartplot(Times, u);
% smartplot(Times, pitchRate);
% smartplot(Times, WeightedYawRateError);
% smartplot(Times, WeightedYawAngleError);
% smartplot(Times, N);
% smartplot(Times, yaw);
% smartplot(Times, yawRate);
% smartplot(Times, wind);
% smartplot(Times, rgv1moving);

function smartplot(t, y)
    plot(t, y, 'DisplayName', inputname(2));
    legend(Location="best");
end
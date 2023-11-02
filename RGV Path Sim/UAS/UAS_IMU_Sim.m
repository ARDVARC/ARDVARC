%% Simulate IMU Measurements
% New preliminary script to mess with matlab imu object
% Does not run in a loop

close all; clc;
tic
% Create a gyroscope object
gyroscope = gyroparams;
% Create an accelerometer object
accelerometer = accelparams;

% Time to run sim for
t_elapsed = 10; % seconds
% Sampling rate
Fs = 100; % Hz
% Number of samples generated
N = t_elapsed*Fs + 1;
% Generate time vector
t = (0:(1/Fs):(t_elapsed))';

% Frequency of test sine wave
Fc = 0.25; % Hz

% Preallocate accelerometer ground truth
% Treat Z axis as any additional acceleration in addition to gravity as imu
% object automatically accounts for gravity
acc = zeros(N, 3);
% Preallocate ground truth
angvel = zeros(N, 3);
% Create Test sine wave (for ground truth)
angvel(:,1) = sin(2*pi*Fc*t);

% Create IMU sensor object
imu = imuSensor('SampleRate', Fs, 'Gyroscope', gyroscope, ...
    'Accelerometer', accelerometer);

% Introduce a bias to the gyroscope
imu.Gyroscope.ConstantBias = 0.1;
% Introduce noise to the gyroscope
imu.Gyroscope.NoiseDensity = 0.01;

% Introduce noise to accelerometer
imu.Accelerometer.NoiseDensity = 0.001;

% Run simulation
[accData, gyroData] = imu(acc, angvel);

% Add gravitational acc to ground truth acc vector
acc(:,3) = acc(:,3) + 9.81;

toc
%% Plots 
% Plot the x gyro data over time
figure
p = plot(t, angvel(:,1), '--', t, gyroData(:,1));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Gyroscope X axis readings over time')
legend('x (Ground Truth)', 'X (Gyroscope)')

% Plot the Z accelerometer data over time
figure
p = plot(t, acc(:,3), '--', t, accData(:,3));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Accelerometer Z axis readings over time ')
ylim([0 10])
legend('x (Ground Truth)', 'Z (Accelerometer)')


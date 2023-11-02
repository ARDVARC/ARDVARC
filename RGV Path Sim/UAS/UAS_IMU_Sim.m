%% Simulate IMU Measurements
% New preliminary script to mess with matlab imu object
% Does not run in a loop

close all; clc;
tic
%% Create sensor objects
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

% Frequency of test sine wave for gyro x axis
Fc = 0.25; % Hz

% Preallocate accelerometer ground truth
% Treat Z axis as any additional acceleration in addition to gravity as imu
% object automatically accounts for gravity
acc = zeros(N, 3);
% Preallocate ground truth
angvel = zeros(N, 3);
% Create Test sine wave (for ground truth)
angvel(:,1) = sin(2*pi*Fc*t);

%%  Create IMU sensor object
imu = imuSensor('SampleRate', Fs, 'Gyroscope', gyroscope, ...
    'Accelerometer', accelerometer);

%% Add in some non-ideal properties to the sensors (arbitrary values for now)
% Introduce a bias to the gyroscope
imu.Gyroscope.ConstantBias = 0.1; %rad/s
% Introduce noise to the gyroscope
imu.Gyroscope.NoiseDensity = 0.001;
% Give the gyro a non-ideal resolution
imu.Gyroscope.Resolution = 0.1; %rad/s / LSB

% Introduce noise to accelerometer
imu.Accelerometer.NoiseDensity = 0.001;


%% Run simulation
[accData, gyroData] = imu(acc, angvel);

% Add gravitational acc to ground truth acc vector
acc(:,3) = acc(:,3) + 9.81;

toc
%% Plots 
% Plot the gyro data over time
figure
subplot(3,1,1);
p = plot(t, angvel(:,1), '--', t, gyroData(:,1));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Gyroscope X axis readings over time')
legend('Ground Truth', 'Gyroscope')

subplot(3,1,2); 
p = plot(t, angvel(:,2), '--', t, gyroData(:,2));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Gyroscope Y axis readings over time')
legend('Ground Truth', 'Gyroscope')

subplot(3,1,3);
p = plot(t, angvel(:,3), '--', t, gyroData(:,3));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Gyroscope Z axis readings over time')
legend('Ground Truth', 'Gyroscope')

% Plot the Z accelerometer data over time
figure
subplot(3,1,1);
p = plot(t, acc(:,1), '--', t, accData(:,1));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Accelerometer X axis readings over time ')
legend('Ground Truth', 'Accelerometer')

subplot(3,1,2);
p = plot(t, acc(:,2), '--', t, accData(:,2));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Accelerometer Y axis readings over time ')
legend('Ground Truth', 'Accelerometer')

subplot(3,1,3);
p = plot(t, acc(:,3), '--', t, accData(:,3));
p(1).LineWidth = 2;
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Accelerometer Z axis readings over time ')
ylim([0 10])
legend('Ground Truth', 'Accelerometer')


function posNodeTrackEst = helperBLEKalmanFilter(motionModel,posNodeEst)
%helperBLEKalmanFilter Tracks the Bluetooth LE node using linear Kalman
%filter
%   POSNODETRACKEST = helperBLEKalmanFilter(MOTIONMODEL,POSNODEEST) tracks
%   the Bluetooth LE node using linear Kalman filter.
%
%   POSNODETRACKEST is a vector of size 2-by-M or 3-by-M, represents the M
%   estimated node positions using linear Kalman filter. Each column of
%   POSNODEEST denotes the 2-D or 3-D position of estimated node.
%
%   MOTIONMODEL is a character vector, represents the linear motion model.
%   This value must be one of '2-D Constant Velocity', '3-D Constant
%   Velocity', '2-D Constant Acceleration', '3-D Constant Acceleration'.
%
%   POSNODEEST is a vector of size 2-by-M or 3-by-M, represents the M
%   estimated node positions in a network using angulation or
%   distance-angle methods. Each column of POSNODEEST denotes the 2-D or
%   3-D position of estimated node.

%   Copyright 2021-2022 The MathWorks, Inc.

modelsplit = strsplit(motionModel,'-');
modelType = strcat(modelsplit(1),modelsplit(2));

% Create and configure Kalman filter
KF = trackingKF('MotionModel',modelType);

% Derive initial state based on the initial location and motion model
if all(isnan(posNodeEst(:,1)))
    initialLocation = zeros(size(posNodeEst,1),1);
else
    initialLocation = posNodeEst(:,1);
end

if strcmp(motionModel,'2-D Constant Velocity')
    gap = 2;
    initialState = [initialLocation(1); 0;initialLocation(2);0];
elseif strcmp(motionModel,'3-D Constant Velocity')
    gap = 2;
    initialState = [initialLocation(1); 0;initialLocation(2);0;initialLocation(3);0];
elseif strcmp(motionModel,'2-D Constant Acceleration')
    gap = 3;
    initialState = [initialLocation(1); 0;0;initialLocation(2);0;0];
else
    gap = 3;
    initialState = [initialLocation(1); 0;0;initialLocation(2);0;0;initialLocation(3);0;0];
end
KF.State=initialState;

% Estimate the node using Kalman filter. If node estimation measurement is
% not available (all the node-locators links fail) then only call the
% predict method. If node estimation measurement is available then call
% predict and correct methods together.
[pstates,cstates] = deal(zeros(size(posNodeEst,1)*2,size(posNodeEst,2)));
posNodeTrackEst = zeros(size(posNodeEst));
for z = 1:size(posNodeEst,2)
    pstates(:,z) = predict(KF,0.25);
    if isnan(posNodeEst(:,z))
        posNodeTrackEst(:,z) = pstates(1:gap:end,z);
    else
        cstates(:,z) = correct(KF,posNodeEst(:,z));
        posNodeTrackEst(:,z) = cstates(1:gap:end,z);
    end
end
end
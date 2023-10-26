function [posNode,posLocators] = helperBLEPositions(motionModel,numNodePositions,numLocators)
%helperBLEPositions Generates locators and node positions
%   [POSNODE,POSLOCATORS] = helperBLEPositions(MOTIONMODEL,
%   NUMNODEPOSITIONS,NUMLOCATORS) generates Bluetooth LE node and locator
%   positions in 2-D or 3-D space.
%
%   POSNODE is a vector of size 2-by-M or 3-by-M, represents the M node
%   positions in a network. Each column of POSNODE denotes the 2-D or 3-D
%   position of node.
%
%   POSLOCATORS is a matrix of size 2-by-N or 3-by-N, represents the
%   position of the N number of locators in a network. Each column of
%   POSLOCATORS denotes the 2-D or 3-D position of each locator.
%
%   MOTIONMODEL is a character vector, represents the linear motion model.
%   This value must be one of '2-D Constant Velocity', '3-D Constant
%   Velocity', '2-D Constant Acceleration', '3-D Constant Acceleration'.
%
%   NUMNODEPOSITIONS is a scalar, represents the number of node positions
%   to track in a network.
%
%   NUMLOCATORS is a scalar, represents the number of locators in a
%   network.

%   Copyright 2021 The MathWorks, Inc.

T = 0.25; % Time step

% Generate locators in all the four quadrants
const = ceil(numLocators/4);
posLocators = [randsrc(1,const,-70:-1) randsrc(1,const,-70:-1) randsrc(1,const,1:70) randsrc(1,numLocators-3*const,1:70);...
               randsrc(1,const,1:70) randsrc(1,const,-70:-1) randsrc(1,const,1:70) randsrc(1,numLocators-3*const,-70:-1)]; 
if strcmp(motionModel,'2-D Constant Velocity') || strcmp(motionModel,'3-D Constant Velocity')
    i = -numNodePositions/2+1:1:numNodePositions/2;
    % Velocities in x, y and z directions in m/s
    vx = 4;
    vy = 6;
    vz = 2;
    if strcmp(motionModel,'2-D Constant Velocity')
        posNode = [vx*T*i; vy*T*i];
    else
        posNode = [vx*T*i; vy*T*i;vz*T*i];
        posLocators(3,:) = [randsrc(1,const,-50:-1) randsrc(1,const,-50:-1)...
                    randsrc(1,const,1:50) randsrc(1,numLocators-3*const,1:50)];
    end
else
    i = 0:numNodePositions-1;
    % Acceleration in x, y and z directions in m2/s
    ax = 1;
    ay = 2;
    az = 0.5;
    if strcmp(motionModel,'2-D Constant Acceleration')
        posNode = [0.5*ax*(T*i).^2; 0.5*ay*(T*i).^2];
    else
        posNode = [0.5*ax*(T*i).^2; 0.5*ay*(T*i).^2;0.5*az*(T*i).^2];
        posLocators(3,:) = [randsrc(1,const,-50:-1) randsrc(1,const,-50:-1)...
                    randsrc(1,const,1:50) randsrc(1,numLocators-3*const,1:50)];
    end
end
end
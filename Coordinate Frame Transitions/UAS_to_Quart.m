% Timothy Behrer
% ASEN 3801
% Quarternion for UAS to Inertial
% Created: 10/27/23
function [q] = UAS_to_Quart(Yaw,Pitch,Roll)
% Input: 
%   Yaw - Ideal, angle around axis 3 in radians
%   Pitch - Ideal, angle around axis 2 in radians
%   Roll - Ideal, angle around axis 1 in radians
%
% Output: 
%           Quarternion_UI - Quarternions
% 
% 
% Methodology: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find Quarternion Values
%Establish used relations
cr = cos(Roll/2);
sr = sin(Roll/2);
cp = cos(Pitch/2);
sp = sin(Pitch/2);
cy = cos(Yaw/2);
sy = sin(Yaw/2);

%Calculate quartenian values
qw = cr * cp * cy + sr * sp * sy;
qx = sr * cp * cy - cr * sp * sy;
qy = cr * sp * cy + sr * cp * sy;
qz = cr * cp * sy - sr * sp * cy;

q = [qw,qx,qy,qz];

end
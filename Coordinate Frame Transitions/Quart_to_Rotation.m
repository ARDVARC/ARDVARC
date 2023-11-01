% Timothy Behrer
% ASEN 3801
% Quarternion to rotation matrix
% Created: 10/27/23
function [RotMat] = Quart_to_Rotation(Quart)
% Input: 
%   Quart - Quarternian vector  
%
% Output: 
%       RotMat - Rotation Matrix
% 
% 
% Methodology: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extract the values from Quart
    q1 = Quart(1);
    q2 = Quart(2);
    q3 = Quart(3);
    q4 = Quart(4);
     
% Individual indexes for rotation matrix
    M11 = 2 * (q1 * q1 + q2 * q2) - 1;
    M12 = 2 * (q2 * q3 - q1 * q4);
    M13 = 2 * (q2 * q4 + q1 * q3);
     
    M21 = 2 * (q2 * q3 + q1 * q4);
    M22 = 2 * (q1 * q1 + q3 * q3) - 1 ;
    M23 = 2 * (q3 * q4 - q1 * q2);
     
    M31 = 2 * (q2 * q4 - q1 * q3);
    M32 = 2 * (q3 * q4 + q1 * q2);
    M33 = 2 * (q1 * q1 + q4 * q4) - 1;
     
% Compile matrix together
    RotMat = [M11, M12, M13; M21, M22, M23; M31, M32, M33];
end
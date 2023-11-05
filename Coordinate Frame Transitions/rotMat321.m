% Timothy Behrer
% ARDVARC
% Rotation Matrix function
% Created: 11/04/23
function rotMat = rotMat321(attitude)
% Input:
% % % attitude = 3 x 1 vector with the 3-2-1 Euler angles of the camera in the form of relative attitude
% % %          = [theta_1_cam2uasBody,theta_2_cam2uasBody,theta_3_cam2uasBody]
% Output: 
% % % rotMat321 = the 3-2-1 rotation matrix parameterized by those angles.
% Methodology: This function converts the camera coordinate frame to the uas body camera frame using a 321 rotation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%% Create the Rotation Matrix 
% 321 Rotation Matrix 
rotMat = [cos(attitude(2))*cos(attitude(3)), cos(attitude(2))*sin(attitude(3)), -sin(attitude(2)); ...
        -cos(attitude(1))*sin(attitude(3)) + sin(attitude(1))*sin(attitude(2))*cos(attitude(3)), cos(attitude(1))*cos(attitude(3)) + sin(attitude(1))*sin(attitude(2))*sin(attitude(3)), sin(attitude(1))*cos(attitude(2)); ...
        sin(attitude(1))*sin(attitude(3)) + cos(attitude(1))*sin(attitude(2))*cos(attitude(3)), -sin(attitude(1))*cos(attitude(3)) + cos(attitude(1))*sin(attitude(2))*sin(attitude(3)), cos(attitude(1))*cos(attitude(2))];



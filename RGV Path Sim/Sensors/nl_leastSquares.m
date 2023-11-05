% Timothy Behrer
% ARDVARC
% Least Squares position estimation function
% Created: 11/04/23
function vec_uas2obj_in_uasBody = nl_leastSquares(vec_uasState,vec_azimuth_in_cam)
%% Create a function that takes in an applicable vecor of azimuth measurements and ....
% derives an expected position using non-linear least squares method
% This function should effectively act as a low-level course localization
% estimator

%%% Current assumptions - the target being measured is not moving
%%%                     - the camera azimuth determined is perfect
%%%                     - the uasState vector is perfect
%TODO(TB) - encorperate a constant moving target
%TODO(TB) - encorperate an accelerating target


%% Start by using coordinate frame transitions to transfer from camera frame to UAS relative frame

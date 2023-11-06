% Timothy Behrer
% ARDVARC
% Least Squares position estimation function
% Created: 11/04/23
function vec_uas2obj_in_uasBody = nl_leastSquares(vec_uasState_pos,vec_uasState_att,vec_targetPoint_in_cam)
%% Create a function that takes in an applicable vecor of azimuth measurements and ....
% derives an expected position using non-linear least squares method
% This function should effectively act as a low-level course localization
% estimator
%%INPUT: vec_uasState_pos - 3 x n matrix of the uasState position
%        vec_uasState_att - 3 x n matrix of the uasState attitude
%        vec_targetPoint_in_cam - 3 x n matrix of the pointing vectors
%%% Current assumptions - the target being measured is not moving
%%%                     - the camera azimuth determined is perfect
%%%                     - the uasState vector is perfect
%%%                     - the uasState vector is a perfect circle around the target
%TODO(TB) - encorperate a constant moving target
%TODO(TB) - encorperate an accelerating target


%% Start by using coordinate frame transitions to transfer from camera frame to UAS relative frame
%Preallocation
size_array = size(vec_targetPoint_in_cam);
vec_targetLine_in_enu = cell(size(vec_targetPoint_in_cam));
vec_targetPoint_in_uasBody = zeros(size(vec_targetPoint_in_cam));
vec_targetPoint_in_enu = zeros(size(vec_targetPoint_in_cam));
trix_extrapLine = linspace(1,100,100);
trix_extrapTemp = zeros(3,length(trix_extrapLine));

for i = 1:size_array(2)
    vec_targetPoint_in_uasBody(:,i) = cam.cam2uasBody(vec_targetPoint_in_cam(:,i)');
    vec_targetPoint_in_enu(:,i) = eul2rotm(vec_uasState_att(:,i)') * vec_targetPoint_in_uasBody(:,i);
    %% Extrapolate the line out
    for j = 1:length(trix_extrapLine)
         trix_extrapTemp(:,j) = vec_targetPoint_in_enu(:,i) * trix_extrapLine(j);
    end
    vec_targetLine_in_enu{i} = trix_extrapTemp;
    
end

plot(vec_targetLine_in_enu{1}(1,:),vec_targetLine_in_enu{1}(2,:))
xlabel('East')
ylabel('North')



















%% TODO - Input truth values here
classdef cam
    properties(Constant)
        %Define constant properties
        %% Extrinsics
        theta_1_cam2uasBody (1,1) double = -pi/4; %Rads **Not a True Value**
        theta_2_cam2uasBody (1,1) double = 0; %Rads **Not a True Value**
        theta_3_cam2uasBody (1,1) double = -pi/2; %Rads **Not a True Value** 
        vec_uasBody2cam (3,1) double = [-.24, 0, -0.1]'; %meters **Not a True Value**
        %321 Rotation Matrix 
        dcm = [cos(cam.theta_2_cam2uasBody)*cos(cam.theta_3_cam2uasBody), cos(cam.theta_2_cam2uasBody)*sin(cam.theta_3_cam2uasBody), -sin(cam.theta_2_cam2uasBody); ...
            -cos(cam.theta_1_cam2uasBody)*sin(cam.theta_3_cam2uasBody) + sin(cam.theta_1_cam2uasBody)*sin(cam.theta_2_cam2uasBody)*cos(cam.theta_3_cam2uasBody), cos(cam.theta_1_cam2uasBody)*cos(cam.theta_3_cam2uasBody) + sin(cam.theta_1_cam2uasBody)*sin(cam.theta_2_cam2uasBody)*sin(cam.theta_3_cam2uasBody), sin(cam.theta_1_cam2uasBody)*cos(cam.theta_2_cam2uasBody); ...
            sin(cam.theta_1_cam2uasBody)*sin(cam.theta_3_cam2uasBody) + cos(cam.theta_1_cam2uasBody)*sin(cam.theta_2_cam2uasBody)*cos(cam.theta_3_cam2uasBody), -sin(cam.theta_1_cam2uasBody)*cos(cam.theta_3_cam2uasBody) + cos(cam.theta_1_cam2uasBody)*sin(cam.theta_2_cam2uasBody)*sin(cam.theta_3_cam2uasBody), cos(cam.theta_1_cam2uasBody)*cos(cam.theta_2_cam2uasBody)];
        
        %% Intrinsics
        scaleFactor (1,1) double = 1; %**Not a True Value**
        focalLength (1,1) double = NaN; %**Not a True Value**
        principalPoint (1,1) double = NaN; %**Not a True Value**
        skew (1,1) double = NaN; %**Not a True Value**
        

    end

    %%Create a method converting a radius to the uasBody frame
    methods(Static)
        function vec_in_uasBody = cam2uasBody(vec)
            %vec - a 3x1 vector in camera frame coordinates
            %vec_in_uasBody - the same 3x1 vector in uasBody coordinates
            vec_in_uasBody = cam.dcm * vec';
        end
    end
end

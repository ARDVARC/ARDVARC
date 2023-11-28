%% TODO - Input truth values here
classdef CamParams
    properties(Constant)
        %Define constant properties
        %% Extrinsics
        theta_1_cam2uasBody (1,1) double = -pi/4; %Rads **Not a True Value**
        theta_2_cam2uasBody (1,1) double = 0; %Rads **Not a True Value**
        theta_3_cam2uasBody (1,1) double = -pi/2; %Rads **Not a True Value** 
        vec_uasBody2cam (3,1) double = [-.24, 0, -0.1]'; %meters **Not a True Value**
        %321 Rotation Matrix 
        dcm = [cos(CamParams.theta_2_cam2uasBody)*cos(CamParams.theta_3_cam2uasBody), cos(CamParams.theta_2_cam2uasBody)*sin(CamParams.theta_3_cam2uasBody), -sin(CamParams.theta_2_cam2uasBody); ...
            -cos(CamParams.theta_1_cam2uasBody)*sin(CamParams.theta_3_cam2uasBody) + sin(CamParams.theta_1_cam2uasBody)*sin(CamParams.theta_2_cam2uasBody)*cos(CamParams.theta_3_cam2uasBody), cos(CamParams.theta_1_cam2uasBody)*cos(CamParams.theta_3_cam2uasBody) + sin(CamParams.theta_1_cam2uasBody)*sin(CamParams.theta_2_cam2uasBody)*sin(CamParams.theta_3_cam2uasBody), sin(CamParams.theta_1_cam2uasBody)*cos(CamParams.theta_2_cam2uasBody); ...
            sin(CamParams.theta_1_cam2uasBody)*sin(CamParams.theta_3_cam2uasBody) + cos(CamParams.theta_1_cam2uasBody)*sin(CamParams.theta_2_cam2uasBody)*cos(CamParams.theta_3_cam2uasBody), -sin(CamParams.theta_1_cam2uasBody)*cos(CamParams.theta_3_cam2uasBody) + cos(CamParams.theta_1_cam2uasBody)*sin(CamParams.theta_2_cam2uasBody)*sin(CamParams.theta_3_cam2uasBody), cos(CamParams.theta_1_cam2uasBody)*cos(CamParams.theta_2_cam2uasBody)];
    end
    properties(Access=public)
        %% Intrinsics
        scaleFactor (1,1) double = 1; %**Not a True Value**
        focalLength (1,1) double = .002; %**Not a True Value**
        principalPoint (1,1) double = NaN; %**Not a True Value**
        skew (1,1) double = NaN; %**Not a True Value**
        Height (1,1) double = 200; %**Not a True Value**
        Width (1,1) double = 400; %**Not a True Value**
        rho_h (1,1) double = 1.4e-6;
        rho_w (1,1) double = 1.4e-6;
        
      
    end

    %%Create a method converting a radius to the uasBody frame
    methods(Static)
        function vec_in_uasBody = cam2uasBody(vec)
            %vec - a 3x1 vector in camera frame coordinates
            %vec_in_uasBody - the same 3x1 vector in uasBody coordinates
            %%TODO incorperate radius value
            vec_in_uasBody = CamParams.dcm * vec';
        end
    end
end

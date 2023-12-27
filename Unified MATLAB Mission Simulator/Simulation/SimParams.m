classdef SimParams
    % A large collection of simulation parameters that can be changed each
    % run. This should prevent the need to recompile for small parameter
    % tweaks
    properties
        % Mission Area Details
        missionAreaHalfWidth = 22.86;

        % UAS Physical Properties
        m = 5.3063;
        Ix = 5.3063 * 0.3475^2 / 4;
        Iy = 5.3063 * 0.3475^2 / 4;
        Iz = 5.3063 * 0.3475^2 / 2;
        LMNmax = 22.63 * 0.3475;
        Zmax = 73.54;
        
        % UAS Control Parameters
        targetUasHeight = 10;
        targetRgvTrailDistance = 1;
        flightPlanRate = 1;
        orbitDuration = 30;
        orbitRadius = 3;
        jointFocusTime = 30;
        localizeTargetDuration = 90;
        localizeMinDuration = 60;

        % RGV Location Prediction
        splineCount = 3;
        nodeGap = 8;
        stillSpeedTolerance = 0.3;

        % UAS Sensor Parameters
        bluetoothRate = 5;
        bluetoothAngleStdRad = deg2rad(5);
        dcm_bluetooth2uas = eul2rotm([pi, 0.2, 0.2]);
        
        % Simulation Parameters
        rgv1Seed (1,1) double 
        rgv2Seed (1,1) double
        duration (1,1) double = 15*60;
        sampleRate = 10;
        vec_uasStartState (12,1) double;
        vec_rgv1startPos_en (2,1) double;
        rgv1startYawAngle (1,1) double = pi/2;
        vec_rgv2startPos_en (2,1) double;
        rgv2startYawAngle (1,1) double = -pi/2;

        % RGV Parameters
        rgvParams (1,1) RgvParams;
    end

    methods
        function this = SimParams()
            this.rgv1Seed = randi(intmax);
            this.rgv2Seed = randi(intmax);
            this.vec_uasStartState = [-this.missionAreaHalfWidth;0;-this.targetUasHeight;zeros(9,1)];
            this.vec_rgv1startPos_en = [0;-this.missionAreaHalfWidth+4];
            this.vec_rgv2startPos_en = [0;this.missionAreaHalfWidth-4];
            this.rgvParams = RgvParams();
        end
    end
end
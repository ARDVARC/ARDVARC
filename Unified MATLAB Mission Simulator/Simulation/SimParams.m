classdef SimParams
    % A large collection of simulation parameters that can be changed each
    % run. This should prevent the need to recompile for small parameter
    % tweaks
    properties
        % Mission Area Details
        missionAreaHalfWidth = 22.86;

        % UAS Physical Properties
        m = 1;
        Ix = 1;
        Iy = 1;
        Iz = 1;
        LMNmax = 10;
        Zmax = 50;
        
        % UAS Control Parameters
        targetUasHeight = 10;
        targetRGVgroundDistance = 3;

        % Mission Parameters
        idealJointDuration = 20;

        % Simulation Parameters
        rgv1Seed (1,1) double 
        rgv2Seed (1,1) double
        duration (1,1) double = 30*60;
        sampleRate = 10;
        uasStartState (12,1) double;
        rgv1startPos (3,1) double;
        rgv1startEul (3,1) double = [pi/2;0;0];
        rgv2startPos (3,1) double;
        rgv2startEul (3,1) double = [-pi/2;0;0];

        % RGV Parameters
        rgvParams = RgvParams();
    end

    methods
        function this = SimParams()
            this.rgv1Seed = randi(10000);
            this.rgv2Seed = randi(10000);
            this.uasStartState = [-this.missionAreaHalfWidth;0;-this.targetUasHeight;zeros(9,1)];
            this.rgv1startPos = [0;-this.missionAreaHalfWidth+4;0];
            this.rgv2startPos = [0;this.missionAreaHalfWidth-4;0];
        end
    end
end
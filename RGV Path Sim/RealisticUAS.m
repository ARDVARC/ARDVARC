classdef RealisticUAS
    properties(Access=private)
        m = 1;
        Ix = 1;
        Iy = 1;
        Iz = 1;
        targetHeight = 10;
        LMNmax = 10;
        Zmax = 50;
        targetRGVgroundDistance = 3;
    end

    methods
        function [ts, positions] = simulate(this, duration, startPos, rgv1, rgv2)
            arguments(Input)
                this (1,1) RealisticUAS
                duration (1,1) double = 30*60
                startPos (3,1) double = [Simulation.missionAreaHalfWidth;Simulation.missionAreaHalfWidth;this.targetHeight]
                rgv1 (1,1) RGV = RGV.makeFromSeed(randi(10000), [-5 5 0], [0 0 0], duration)
                rgv2 (1,1) RGV = RGV.makeFromSeed(randi(10000), [-5 5 0], [0 0 0], duration)
            end
            arguments(Output)
                ts (:,1) double
                positions (:,3) double
            end
            opts = odeset("AbsTol",1e-6);
            [ts, states] = ode45(@(t,state) this.EOM(t, state, rgv1, rgv2), [0 duration], [startPos(1:2);-startPos(3);zeros(9,1)],opts);
            positions = states(:,1:3);
        end
    end

    methods(Access=private)
        function dState = EOM(this, t, state, rgv1, rgv2)
            arguments(Input)
                this (1,1) RealisticUAS
                t (1,1) double
                state (12,1) double
                rgv1 (1,1) RGV
                rgv2 (1,1) RGV
            end
            arguments(Output)
                dState (12,1) double
            end
            pos = state(1:3);
            rgv1_pos = rgv1.getStateAtTime(t)';
            rgv2_pos = rgv2.getStateAtTime(t)';
            rgv1_dist = norm(pos-rgv1_pos);
            rgv2_dist = norm(pos-rgv2_pos);
            if (rgv1_dist < rgv2_dist)
                RGVxyz = rgv1_pos + [0;0;this.targetHeight];
            else
                RGVxyz = rgv2_pos + [0;0;this.targetHeight];
            end
            
            phi = state(4);
            theta = state(5);
            psi = state(6);
            uvw = state(7:9);
            p = state(10);
            q = state(11);
            r = state(12);
            pqr = state(10:12);
        
            RB2E = this.getRB2E(state);
            rotDynMat = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);
                         0,cos(phi)           ,-sin(phi)          ;
                         0,sin(phi)*sec(theta),cos(phi)*sec(theta)];
        
            [Lc,Mc,Nc,Zc] = this.getLMNZ(t, state, RB2E, RGVxyz);
            xyzdot = RB2E*uvw;
        
            phithetapsidot = rotDynMat*pqr;
        
            gravDir = [-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)];
            % TODO - Aerodynamic forces
            uvwdot = cross(uvw,pqr) + 9.81*gravDir + [0;0;Zc]/this.m;
            
            % TODO - Aerodynamic moments
            pqrdot = [(this.Iy-this.Iz)/this.Ix*q*r;(this.Iz-this.Ix)/this.Iy*p*r;(this.Ix-this.Iy)/this.Iz*p*q] + [Lc/this.Ix;Mc/this.Iy;Nc/this.Iz];
        
            dState = [xyzdot;phithetapsidot;uvwdot;pqrdot];
        end
        function [rollError, pitchError, yawError, toNearRGVB] = getUsefulAngles(this, state, RB2E, RGVxyz)
            arguments(Input)
                this (1,1) RealisticUAS
                state (12,1) double
                RB2E (3,3) double
                RGVxyz (3,1) double
            end
            
            xyz = state(1:3);
            RE2B = RB2E^-1;
            
            headingB = [1;0;0];
            headingE = RB2E*headingB;
        
            toAboveRGVE = RGVxyz - xyz + [0;0;-this.targetHeight];
            flatToAboveRGVE = normalize(toAboveRGVE(1:2));
            toNearRGVE = toAboveRGVE - this.targetRGVgroundDistance*[flatToAboveRGVE;0];
        
            desiredHeadingE = normalize(toNearRGVE);
            toNearRGVB = RE2B*toNearRGVE;
            rightB = [0;1;0];
            rightE = RB2E*rightB;
            flatHeadingE = normalize(headingE(1:2));
            flatDesiredHeadingE = normalize(desiredHeadingE(1:2));
        
            pitchError = asin(headingE(3));
            yawError = signedAngle(flatDesiredHeadingE, flatHeadingE);
            rollError = -asin(rightE(3));
        end
        function [L,M,N,Z] = getLMNZ(this, t, state, RB2E, RGVxyz)
            z = state(3);
            u = state(7);
            v = state(8);
            w = state(9);
            p = state(10);
            q = state(11);
            r = state(12);
        
            [rollError, pitchError, yawError, toNearRGVB] = this.getUsefulAngles(state, RB2E, RGVxyz);
        
            L = 2000*rollError-80*v-200*p+140*clamp(toNearRGVB(2),-5,5);
            M = 2000*pitchError+80*u-200*q-140*clamp(toNearRGVB(1),-5,5);
            N = -1200*yawError-200*r;
            Z = -9.81/cos(pitchError)/cos(rollError)+2*-clamp(z+this.targetHeight,-5,5)-4*clamp(w,-1,1);
        
            L = clamp(L,-this.LMNmax,this.LMNmax);
            M = clamp(M,-this.LMNmax,this.LMNmax);
            N = clamp(N,-this.LMNmax,this.LMNmax);
            Z = clamp(Z,-this.Zmax,this.Zmax);
        end
    end

    methods(Static)
        function RB2E = getRB2E(state)
            phi = state(4);
            theta = state(5);
            psi = state(6);
            RB2E = [cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                    cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                    -sin(theta)        ,sin(phi)*cos(theta)                           ,cos(phi)*cos(theta)                           ];
        end
    end
end
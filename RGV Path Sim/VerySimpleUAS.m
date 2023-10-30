classdef VerySimpleUAS
    properties(Access=private)
        k1 (1,1) double
        k2 (1,1) double
        max_accel (1,1) double
    end

    methods
        function this = VerySimpleUAS(k1, k2, max_accel)
            arguments(Input)
                k1 (1,1) double = 1
                k2 (1,1) double = 1
                max_accel (1,1) double = 5
            end
            arguments(Output)
                this (1,1) VerySimpleUAS
            end

            this.k1 = k1;
            this.k2 = k2;
            this.max_accel = max_accel;
        end

        function [ts, positions] = simulate(this, duration, startPos2D, height, rgv1, rgv2)
            arguments(Input)
                this (1,1) VerySimpleUAS
                duration (1,1) double = 30*60
                startPos2D (2,1) double = [Simulation.missionAreaHalfWidth;Simulation.missionAreaHalfWidth]
                height (1,1) double = 10
                rgv1 (1,1) RGV = RGV.makeFromSeed(randi(10000), [-5 5 0], [0 0 0], duration)
                rgv2 (1,1) RGV = RGV.makeFromSeed(randi(10000), [-5 5 0], [0 0 0], duration)
            end
            arguments(Output)
                ts (:,1) double
                positions (:,3) double
            end
            [ts, states] = ode45(@(t,state) this.foo(t, state, rgv1, rgv2), [0 duration], [startPos2D;height;0;0;0]);
            positions = states(:,1:3);
        end
    end

    methods(Access=private)
        function dState = foo(this, t, state, rgv1, rgv2)
            arguments(Input)
                this (1,1) VerySimpleUAS
                t (1,1) double
                state (6,1) double
                rgv1 (1,1) RGV
                rgv2 (1,1) RGV
            end
            arguments(Output)
                dState (6,1) double
            end
            pos = state(1:3);
            vel = state(4:6);
            rgv1_pos = rgv1.getStateAtTime(t)';
            rgv2_pos = rgv2.getStateAtTime(t)';
            rgv1_dist = norm(pos-rgv1_pos);
            rgv2_dist = norm(pos-rgv2_pos);
            if (rgv1_dist < rgv2_dist)
                near_rgv_pos = rgv1_pos + [0;0;pos(3)];
            else
                near_rgv_pos = rgv2_pos + [0;0;pos(3)];
            end
            
            delta = near_rgv_pos - pos;
            dir = delta/norm(delta);
            deltaTarget = delta - 2*dir;
        
            accel = deltaTarget*this.k1 - vel*this.k2;
        
            if (norm(accel) > this.max_accel)
                accel = accel/norm(accel)*this.max_accel;
            end
        
            dState = [vel;accel];
        end
    end
end
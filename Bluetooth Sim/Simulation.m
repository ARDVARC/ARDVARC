classdef Simulation
    properties(Constant)
        missionAreaHalfWidth = 22.86;
    end

    methods(Static)
        function dist = DistanceToBondary(pos)
            dist = min([pos(1) + Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth - pos(1), pos(2) + Simulation.missionAreaHalfWidth, Simulation.missionAreaHalfWidth - pos(2)]);
        end
    end
end
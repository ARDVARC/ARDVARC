function dist = distanceToBoundary(pos)
    % Returns how far a given initial vector is from the mission area
    % boundary. Ignores height
    arguments(Input)
        pos (3,1) double
    end
    arguments(Output)
        dist (1,1) double
    end

    global simParams;
    
    dist = min([pos(1) + simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth - pos(1), pos(2) + simParams.missionAreaHalfWidth, simParams.missionAreaHalfWidth - pos(2)]);
end
function dist = distanceToBoundary(missionAreaHalfWidth, vec_pos_en)
    % Returns how far a given initial vector is from the mission area
    % boundary. Ignores height
    arguments(Input)
        missionAreaHalfWidth (1,1) double
        vec_pos_en (2,1) double
    end
    arguments(Output)
        dist (1,1) double
    end
    
    dist = min([vec_pos_en(1) + missionAreaHalfWidth, missionAreaHalfWidth - vec_pos_en(1), vec_pos_en(2) + missionAreaHalfWidth, missionAreaHalfWidth - vec_pos_en(2)]);
end
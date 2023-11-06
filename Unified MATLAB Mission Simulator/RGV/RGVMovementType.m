classdef RgvMovementType < int8
    % An enum to describe the type of movement path that an RGV is
    % currently moving along
    enumeration
        Wait(0)
        Straight(1)
        ArcLeft(2)
        ArcRight(3)
        UTurnLeft(4)
        UTurnRight(5)
    end
end
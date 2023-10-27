classdef RGVMovementType < Simulink.IntEnumType
    enumeration
        Wait(0)
        Straight(1)
        ArcLeft(2)
        ArcRight(3)
        UTurnLeft(4)
        UTurnRight(5)
    end

    methods(Static)
        function this = getRandom()
            randVal = rand(1);
            if (randVal < RGV.waitProbCutoff)
                this = RGVMovementType.Wait;
            elseif (randVal < RGV.straightProbCutoff)
                this = RGVMovementType.Straight;
            elseif (randVal < RGV.arcLeftCutoff)
                this = RGVMovementType.ArcLeft;
            else
                this = RGVMovementType.ArcRight;
            end
        end
    end
end
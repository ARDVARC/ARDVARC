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
            if (randVal < 0.2)
                this = RGVMovementType.Wait;
            elseif (randVal < 0.7)
                this = RGVMovementType.Straight;
            elseif (randVal < 0.9)
                this = RGVMovementType.ArcLeft;
            else
                this = RGVMovementType.ArcRight;
            end
        end
    end
end
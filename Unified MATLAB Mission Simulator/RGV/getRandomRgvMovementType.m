function this = getRandomRgvMovementType(rgvParams)
    % Returns a random RGV movement type based on the probabiilties
    % assigned to each movement type in rgvParams
    arguments(Input)
        rgvParams (1,1) RgvParams
    end
    arguments(Output)
        this (1,1) RgvMovementType
    end

    randVal = rand(1);
    if (randVal < rgvParams.waitProbCutoff)
        this = RgvMovementType.Wait;
    elseif (randVal < rgvParams.straightProbCutoff)
        this = RgvMovementType.Straight;
    elseif (randVal < rgvParams.arcLeftCutoff)
        this = RgvMovementType.ArcLeft;
    else
        this = RgvMovementType.ArcRight;
    end
end
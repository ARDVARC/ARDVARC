function this = getRandomRgvMovementType(params)
    arguments(Input)
        params (1,1) RgvParams
    end
    arguments(Output)
        this (1,1) RGVMovementType
    end
    randVal = rand(1);
    if (randVal < params.waitProbCutoff)
        this = RGVMovementType.Wait;
    elseif (randVal < params.straightProbCutoff)
        this = RGVMovementType.Straight;
    elseif (randVal < params.arcLeftCutoff)
        this = RGVMovementType.ArcLeft;
    else
        this = RGVMovementType.ArcRight;
    end
end
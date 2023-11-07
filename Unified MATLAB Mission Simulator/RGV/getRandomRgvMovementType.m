function this = getRandomRgvMovementType()
    % Returns a random RGV movement type based on the probabiilties
    % assigned to each movement type in simParams.rgvParams
    arguments(Output)
        this (1,1) RgvMovementType
    end

    global simParams;

    randVal = rand(1);
    if (randVal < simParams.rgvParams.waitProbCutoff)
        this = RgvMovementType.Wait;
    elseif (randVal < simParams.rgvParams.straightProbCutoff)
        this = RgvMovementType.Straight;
    elseif (randVal < simParams.rgvParams.arcLeftCutoff)
        this = RgvMovementType.ArcLeft;
    else
        this = RgvMovementType.ArcRight;
    end
end
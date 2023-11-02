function [goToE, lookAtE] = getGoToAndLookAt(uasState, rgv1Position, rgv2Position)
    arguments(Input)
        uasState (12,1) double
        rgv1Position (3,1) double
        rgv2Position (3,1) double
    end
    arguments(Output)
        goToE (3,1) double
        lookAtE (2,1) double
    end

    global simParams;
    
    uasPosition = uasState(1:3);
    distanceToRgv1 = norm(uasPosition-rgv1Position);
    distanceToRgv2 = norm(uasPosition-rgv2Position);
    if (distanceToRgv1 < distanceToRgv2)
        lookAtE = rgv1Position(1:2);
    else
        lookAtE = rgv2Position(1:2);
    end
    
    pointing = normalize2by1(lookAtE - uasPosition(1:2));
    goToE = [lookAtE - simParams.targetRGVgroundDistance*pointing; -simParams.targetUasHeight];
end
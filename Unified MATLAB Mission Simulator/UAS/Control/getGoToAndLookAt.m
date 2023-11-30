function [goToE, lookAtE] = getGoToAndLookAt(simParams, uasState, rgv1Position, rgv2Position)
    % Determine where the UAS should "go to" and where it should "look at"
    % at a given time step based on estimates of the UAS and RGV states
    arguments(Input)
        simParams (1,1) SimParams
        uasState (12,1) double
        rgv1Position (2,1) double
        rgv2Position (2,1) double
    end
    arguments(Output)
        goToE (3,1) double         % The inertial location that the UAS should physically be
        lookAtE (2,1) double       % The location on the ground that the UAS should be yawed towards
    end
    
    uasPosition = uasState(1:3);
    rgv1Position = [rgv1Position;0];
    rgv2Position = [rgv2Position;0];
    distanceToRgv1 = norm(uasPosition-rgv1Position);
    distanceToRgv2 = norm(uasPosition-rgv2Position);
    if (distanceToRgv1 < distanceToRgv2)
        lookAtE = rgv1Position(1:2);
    else
        lookAtE = rgv2Position(1:2);
    end
    
    pointingE = normalize2by1(lookAtE - uasPosition(1:2));
    goToE = [lookAtE - simParams.targetRGVgroundDistance*pointingE; -simParams.targetUasHeight];
end
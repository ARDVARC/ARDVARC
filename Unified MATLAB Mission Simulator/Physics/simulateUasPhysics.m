function [times, trueUasStates] = simulateUasPhysics(uasStateExtrapolator, rgv1PositionExtrapolater, rgv2PositionExtrapolater, startTime, endTime, trueUasStartState)
    % Uses ode45 to simulate the physics of the UAS over a specified time
    % period and under specified initial conditions
    arguments(Input)
        uasStateExtrapolator (1,1) function_handle      % Function to give a predicted UAS state
        rgv1PositionExtrapolater (1,1) function_handle  % Function to give a predicted position for RGV 1
        rgv2PositionExtrapolater (1,1) function_handle  % Function to give a predicted position for RGV 2
        startTime (1,1) double
        endTime (1,1) double
        trueUasStartState (12,1) double
    end
    opts = odeset("AbsTol",1e-6);
    [times, trueUasStates] = ode45(@(t,trueUasState) ode45Inner(t, trueUasState, uasStateExtrapolator, rgv1PositionExtrapolater, rgv2PositionExtrapolater), [startTime endTime], trueUasStartState, opts);

    function dTrueState = ode45Inner(t, trueUasState, uasStateExtrapolator, rgv1PositionExtrapolater, rgv2PositionExtrapolater)
        % Determine where we think we are and where we think the RGVs are
        extrapolatedUasState = uasStateExtrapolator(t, trueUasState);
        extrapolatedRgv1Position = rgv1PositionExtrapolater(t, extrapolatedUasState, trueUasState);
        extrapolatedRgv2Position = rgv2PositionExtrapolater(t, extrapolatedUasState, trueUasState);
    
        % Determine where we want to go and where we want to be looking
        [goToE, lookAtE] = getGoToAndLookAt(extrapolatedUasState, extrapolatedRgv1Position, extrapolatedRgv2Position);
    
        % Use PD control to determine what forces and moments will get us
        % from where we think we are to where we want to be
        [Lc,Mc,Nc,Zc] = getMomentsAndZForce(extrapolatedUasState, goToE, lookAtE);

        % Do actual physics
        dTrueState = equationsOfMotion(trueUasState, Lc, Mc, Nc, Zc);
    end
end
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
    [times, trueUasStates] = ode45(@(t,trueUasState) equationsOfMotion(t, trueUasState, uasStateExtrapolator, rgv1PositionExtrapolater, rgv2PositionExtrapolater), [startTime endTime], trueUasStartState, opts);
end
function [times, trueUasStates] = simulateUasPhysics(uasStateExtrapolator, rgv1PositionExtrapolater, rgv2PositionExtrapolater, startTime, endTime, trueUasStartState)
    opts = odeset("AbsTol",1e-6);
    [times, trueUasStates] = ode45(@(t,trueUasState) equationsOfMotion(t, trueUasState, uasStateExtrapolator, rgv1PositionExtrapolater, rgv2PositionExtrapolater), [startTime endTime], trueUasStartState, opts);
end
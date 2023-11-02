function uasStateExtrapolator = magicUasStateEstimator()
    % An impossibly perfect UAS state "sensor" that produces an
    % "extrapolater" that just gives the exact UAS state
    arguments(Output)
        uasStateExtrapolator (1,1) function_handle  % Function to give a predicted position for the UAS
    end
    uasStateExtrapolator = @(t, trueState) trueState;
end
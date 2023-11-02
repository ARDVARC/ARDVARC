function uasStateExtrapolator = magicUasStateEstimator()
    arguments(Output)
        uasStateExtrapolator (1,1) function_handle
    end
    uasStateExtrapolator = @(t, trueState) trueState;
end
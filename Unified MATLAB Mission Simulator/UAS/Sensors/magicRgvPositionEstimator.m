function rgvPositionExtrapolater = magicRgvPositionEstimator(rgv)
    arguments(Input)
        rgv (1,1) RGV
    end
    arguments(Output)
        rgvPositionExtrapolater (1,1) function_handle
    end
    rgvPositionExtrapolater = @(t, extrapolatedUasState, trueUasState) rgv.getStateAtTime(t);
end
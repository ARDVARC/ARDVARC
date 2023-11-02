function rgvPositionExtrapolater = magicRgvPositionEstimator(rgv)
    % An impossibly perfect RGV position "sensor" that produces an
    % "extrapolater" that just gives the exact RGV position
    arguments(Input)
        rgv (1,1) RGV
    end
    arguments(Output)
        rgvPositionExtrapolater (1,1) function_handle  % Function to give a predicted position for an RGV
    end
    rgvPositionExtrapolater = @(t, extrapolatedUasState, trueUasState) rgv.getStateAtTime(t);
end
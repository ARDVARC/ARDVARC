function [states, uas] = getUASstatesAtTimes(times, rgv1, rgv2, initialUASpos)
    arguments(Input)
        times (1,:) double
        rgv1 (1,1) RGV
        rgv2 (1,1) RGV
        initialUASpos (3,1) double
    end
    arguments(Output)
        states (:,12) double
        uas (1,1) RealisticUAS
    end
    uas = RealisticUAS.makeAndSimulate(times(end), initialUASpos, rgv1, rgv2);
    states = interp1(uas.uasTimes,uas.uasStates,times);
end
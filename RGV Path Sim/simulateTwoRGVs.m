function [times, rgv1positions, rgv2positions, rgv1movementTypes, rgv2movementTypes, rgv1, rgv2] = simulateTwoRGVs(plotRate, duration, seed1, seed2)
    rgv1 = RGV.makeFromSeed(seed1, [-5 5 0], [0 0 0], duration);
    rgv2 = RGV.makeFromSeed(seed2, [5 -5 0], [pi 0 0], duration);
    
    times = 0:(1/plotRate):duration;
    rgv1positions = zeros(length(times),3);
    rgv2positions = zeros(length(times),3);
    rgv1movementTypes = zeros(length(times),1,"int8");
    rgv2movementTypes = zeros(length(times),1,"int8");
    for i = 1:length(times)
        [rgv1positions(i,:), ~, rgv1movementTypes(i)] = rgv1.getStateAtTime(times(i));
        [rgv2positions(i,:), ~, rgv2movementTypes(i)] = rgv2.getStateAtTime(times(i));
    end
end
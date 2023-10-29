function positions = getRGVpositionAtTimes(rgv, times)
    positions = zeros(length(times), 3);
    for i = 1:length(times)
        positions(i,:) = rgv.getStateAtTime(times(i));
    end
end
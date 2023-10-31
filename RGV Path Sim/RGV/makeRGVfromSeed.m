function rgv = makeRGVfromSeed(seed, startPos, startEul, duration)
    arguments(Input)
        seed (1,1) double
        startPos (3,1) double
        startEul (3,1) double
        duration (1,1) double
    end
    arguments(Output)
        rgv (1,1) RGV
    end
    rgv = RGV.makeFromSeed(seed, startPos, startEul, duration);
end
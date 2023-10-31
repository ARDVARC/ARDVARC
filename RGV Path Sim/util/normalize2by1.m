function normalized = normalize2by1(vec)
    arguments(Input)
        vec (2,1) double
    end
    arguments(Output)
        normalized (2,1) double
    end
    mag = norm(vec);
    if (mag == 0)
        normalized = 0*vec;
    else
        normalized = vec/mag;
    end
end
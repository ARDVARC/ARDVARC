function normalized = normalize2by1(vec)
    % Convert a 3D vector into a unit vector, unless it is the zero vector
    % in which case it doesn't change it.
    arguments(Input)
        vec (2,1) double
    end
    arguments(Output)
        normalized (2,1) double
    end
    mag = norm(vec);
    if (mag == 0)
        normalized = vec;
    else
        normalized = vec/mag;
    end
end
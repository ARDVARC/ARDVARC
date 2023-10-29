function normalized = normalize(vec)
    mag = norm(vec);
    if (mag == 0)
        normalized = 0*vec;
    else
        normalized = vec/mag;
    end
end
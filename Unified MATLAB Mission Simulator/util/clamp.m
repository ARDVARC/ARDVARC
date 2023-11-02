function val = clamp(val, l, u)
    arguments(Input)
        val (1,1) double
        l (1,1) double
        u (1,1) double
    end
    arguments(Output)
        val (1,1) double
    end
    val = min(max(l, val), u);
end
function val = clamp(val, l, u)
    val = min(max(l, val), u);
end
function RB2E = getRB2E(state)
    arguments(Input)
        state (12,1) double
    end
    arguments(Output)
        RB2E (3,3) double
    end
    phi = state(4);
    theta = state(5);
    psi = state(6);
    RB2E = [cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
            cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
            -sin(theta)        ,sin(phi)*cos(theta)                           ,cos(phi)*cos(theta)                           ];
end
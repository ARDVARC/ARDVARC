function dTrueState = equationsOfMotion(trueUasState, Lc, Mc, Nc, Zc)
    % Calculates the rate of change of a UAS state based on the current UAS
    % state and information that can be used to determine the control
    % forces
    arguments(Input)
        trueUasState (12,1) double
        Lc (1,1) double
        Mc (1,1) double
        Nc (1,1) double
        Zc (1,1) double
    end
    arguments(Output)
        dTrueState (12,1) double    % Rate of change for UAS state
    end

    global simParams;

    phi = trueUasState(4);
    theta = trueUasState(5);
    psi = trueUasState(6);
    uvw = trueUasState(7:9);
    p = trueUasState(10);
    q = trueUasState(11);
    r = trueUasState(12);
    pqr = trueUasState(10:12);

    RB2E = getRB2E(trueUasState);
    rotDynMat = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);
                 0,cos(phi)           ,-sin(phi)          ;
                 0,sin(phi)*sec(theta),cos(phi)*sec(theta)];
    xyzdot = RB2E*uvw;

    phithetapsidot = rotDynMat*pqr;

    gravDir = [-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)];
    % TODO - Aerodynamic forces
    uvwdot = cross(uvw,pqr) + 9.81*gravDir + [0;0;Zc]/simParams.m;
    
    % TODO - Aerodynamic moments
    pqrdot = [(simParams.Iy-simParams.Iz)/simParams.Ix*q*r;(simParams.Iz-simParams.Ix)/simParams.Iy*p*r;(simParams.Ix-simParams.Iy)/simParams.Iz*p*q] + [Lc/simParams.Ix;Mc/simParams.Iy;Nc/simParams.Iz];

    dTrueState = [xyzdot;phithetapsidot;uvwdot;pqrdot];
end
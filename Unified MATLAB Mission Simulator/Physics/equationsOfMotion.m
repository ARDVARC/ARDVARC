function dTrueState = equationsOfMotion(t, trueUasState, uasStateExtrapolator, rgv1PositionExtrapolater, rgv2PositionExtrapolater)
    arguments(Input)
        t (1,1) double
        trueUasState (12,1) double
        uasStateExtrapolator (1,1) function_handle
        rgv1PositionExtrapolater (1,1) function_handle
        rgv2PositionExtrapolater (1,1) function_handle
    end
    arguments(Output)
        dTrueState (12,1) double
    end

    global simParams;

    % Determine where we think we are and where we think the RGVs are
    extrapolatedUasState = uasStateExtrapolator(t, trueUasState);
    extrapolatedRB2E = getRB2E(extrapolatedUasState);
    extrapolatedRgv1Position = rgv1PositionExtrapolater(t, extrapolatedUasState, trueUasState);
    extrapolatedRgv2Position = rgv2PositionExtrapolater(t, extrapolatedUasState, trueUasState);

    % Determine where we want to go and where we want to be looking
    [goToE, lookAtE] = getGoToAndLookAt(extrapolatedUasState, extrapolatedRgv1Position, extrapolatedRgv2Position);

    % Use PD control to determine what forces and moments will get us
    % from where we think we are to where we want to be
    [Lc,Mc,Nc,Zc] = getMomentsAndZForce(extrapolatedUasState, extrapolatedRB2E, goToE, lookAtE);
    
    % Do the physics stuff know that we know the forces and moments
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
function [Lc,Mc,Nc,Zc] = getMomentsAndZForce(simParams, uasState, goToE, lookAtE)
    % Determines the various control forces and moments based on where the
    % UAS thinks it is, where it is trying to go, etc. Uses a PD-like
    % control scheme with certain limits/clamps
    arguments(Input)
        simParams (1,1) SimParams
        uasState (12,1) double
        goToE (3,1) double
        lookAtE (2,1) double
    end
    arguments(Output)
        Lc (1,1) double
        Mc (1,1) double
        Nc (1,1) double
        Zc (1,1) double
    end
    
    z = uasState(3);
    u = uasState(7);
    v = uasState(8);
    w = uasState(9);
    p = uasState(10);
    q = uasState(11);
    r = uasState(12);

    uasPosE = uasState(1:3);
    RB2E = getDcmUas2Enu(uasState);
    RE2B = RB2E';
    
    headingB = [1;0;0];
    headingE = RB2E*headingB;

    flatHeadingE = normalize2by1(headingE(1:2));
    fromUasToLookAtE = lookAtE - uasPosE(1:2);
    flatDesiredHeadingE = normalize2by1(fromUasToLookAtE);
    
    rightB = [0;1;0];
    rightE = RB2E*rightB;

    pitchError = asin(headingE(3));
    yawError = signedAngle(flatDesiredHeadingE, flatHeadingE);
    rollError = -asin(rightE(3));

    fromUasToGoToE = goToE - uasPosE;
    fromUasToGoToB = RE2B*fromUasToGoToE;

    Lc = 2000*rollError-200*v-500*p+140*clamp(fromUasToGoToB(2),-5,5);
    Mc = 2000*pitchError+200*u-500*q-140*clamp(fromUasToGoToB(1),-5,5);
    Nc = -1200*yawError-400*r;
    Zc = -9.81/cos(pitchError)/cos(rollError)+2*-5*clamp(z-goToE(3),-0.5,0.5)-4*clamp(w,-1,1);

    Lc = clamp(Lc,-simParams.LMNmax,simParams.LMNmax);
    Mc = clamp(Mc,-simParams.LMNmax,simParams.LMNmax);
    Nc = clamp(Nc,-simParams.LMNmax,simParams.LMNmax);
    Zc = clamp(Zc,-simParams.Zmax,simParams.Zmax);
end
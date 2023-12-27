function [Lc,Mc,Nc,Zc] = getMomentsAndZForce(simParams, vec_uasState, vec_goTo_enu, vec_lookAt_en)
    % Determines the various control forces and moments based on where the
    % UAS thinks it is, where it is trying to go, etc. Uses a PD-like
    % control scheme with certain limits/clamps
    % arguments(Input)
    %     simParams (1,1) SimParams
    %     uasState (12,1) double
    %     goToE (3,1) double
    %     lookAtE (2,1) double
    % end
    % arguments(Output)
    %     Lc (1,1) double
    %     Mc (1,1) double
    %     Nc (1,1) double
    %     Zc (1,1) double
    % end
    
    z = vec_uasState(3);
    phi = vec_uasState(4);
    theta = vec_uasState(5);
    psi = vec_uasState(6);
    u = vec_uasState(7);
    v = vec_uasState(8);
    w = vec_uasState(9);
    p = vec_uasState(10);
    q = vec_uasState(11);
    r = vec_uasState(12);

    uasPosE = vec_uasState(1:3);
    dcm_uas2enu = [cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                   cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                   -sin(theta)        ,sin(phi)*cos(theta)                           ,cos(phi)*cos(theta)                           ];
    dcm_enu2uas = dcm_uas2enu';
    
    headingB = [1;0;0];
    headingE = dcm_uas2enu*headingB;
    
    fromUasToLookAtE = vec_lookAt_en - uasPosE(1:2);
    
    rightB = [0;1;0];
    rightE = dcm_uas2enu*rightB;

    pitchError = asin(headingE(3));
    yawError = atan2(headingE(2)*fromUasToLookAtE(1)-headingE(1)*fromUasToLookAtE(2),headingE(1)*fromUasToLookAtE(1)+headingE(2)*fromUasToLookAtE(2));
    rollError = -asin(rightE(3));

    fromUasToGoToE = vec_goTo_enu - uasPosE;
    fromUasToGoToB = dcm_enu2uas*fromUasToGoToE;

    Lc = 2000*rollError-250*v-500*p+140*min(max(-5, fromUasToGoToB(2)), 5);
    Mc = 2000*pitchError+250*u-500*q-140*min(max(-5, fromUasToGoToB(1)), 5);
    Nc = -1200*yawError-400*r;
    Zc = -9.81*simParams.m/cos(pitchError)/cos(rollError)+2*-5*min(max(-0.5, z-vec_goTo_enu(3)), 0.5)-4*min(max(-1, w), 1);
    
    Lc = min(max(-simParams.LMNmax, Lc), simParams.LMNmax);
    Mc = min(max(-simParams.LMNmax, Mc), simParams.LMNmax);
    Nc = min(max(-simParams.LMNmax, Nc), simParams.LMNmax);
    Zc = min(max(-simParams.Zmax, Zc), simParams.Zmax);
end
function vec_dTrueState = equationsOfMotion(simParams, vec_trueUasState, Lc, Mc, Nc, Zc)
    % Calculates the rate of change of a UAS state based on the current UAS
    % state and information that can be used to determine the control
    % forces
    % arguments(Input)
    %     simParams (1,1) SimParams
    %     vec_trueUasState (12,1) double
    %     Lc (1,1) double
    %     Mc (1,1) double
    %     Nc (1,1) double
    %     Zc (1,1) double
    % end
    % arguments(Output)
    %     vec_dTrueState (12,1) double    % Rate of change for UAS state
    % end

    phi = vec_trueUasState(4);
    theta = vec_trueUasState(5);
    psi = vec_trueUasState(6);
    vec_uvw_uas = vec_trueUasState(7:9);
    p = vec_trueUasState(10);
    q = vec_trueUasState(11);
    r = vec_trueUasState(12);
    vec_pqr_uas = vec_trueUasState(10:12);
    
    dcm_uas2enu = [cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                   cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                   -sin(theta)        ,sin(phi)*cos(theta)                           ,cos(phi)*cos(theta)                           ];
    trix_rotDyn = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);
                   0,cos(phi)           ,-sin(phi)          ;
                   0,sin(phi)*sec(theta),cos(phi)*sec(theta)];
    vec_xyzdot_enu = dcm_uas2enu*vec_uvw_uas;

    vec_phithetapsidot_enu = trix_rotDyn*vec_pqr_uas;

    vec_gravDir_uas = [-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)];
    % TODO - Aerodynamic forces
    vec_uvwdot_uas = cross(vec_uvw_uas,vec_pqr_uas) + 9.81*vec_gravDir_uas + [0;0;Zc]/simParams.m;
    
    % TODO - Aerodynamic moments
    vec_pqrdot_uas = [(simParams.Iy-simParams.Iz)/simParams.Ix*q*r;(simParams.Iz-simParams.Ix)/simParams.Iy*p*r;(simParams.Ix-simParams.Iy)/simParams.Iz*p*q] + [Lc/simParams.Ix;Mc/simParams.Iy;Nc/simParams.Iz];

    vec_dTrueState = [vec_xyzdot_enu;vec_phithetapsidot_enu;vec_uvwdot_uas;vec_pqrdot_uas];
end
function dcm_uas2enu = getDcmUas2Enu(vec_uasState)
    % Calculates a rotation matrix from body frame to inertial frame (based
    % off of a 12 element UAS state vector)
    arguments(Input)
        vec_uasState (12,1) double
    end
    arguments(Output)
        dcm_uas2enu (3,3) double
    end
    phi = vec_uasState(4);
    theta = vec_uasState(5);
    psi = vec_uasState(6);
    dcm_uas2enu = [cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                   cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                   -sin(theta)        ,sin(phi)*cos(theta)                           ,cos(phi)*cos(theta)                           ];
end
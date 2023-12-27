function vec_sensorPointingVec_bluetooth = recordBluetooth(simParams, vec_uasState, vec_rgvPosition_en)
    vec_pointingVecToRgv_enu = vec_uasState(1:3)'-[vec_rgvPosition_en;0];
    mag = norm(vec_pointingVecToRgv_enu);
    if (mag ~= 0)
        vec_pointingVecToRgv_enu = vec_pointingVecToRgv_enu/mag;
    end
    phi = vec_uasState(4);
    theta = vec_uasState(5);
    psi = vec_uasState(6);
    dcm_uas2enu = [cos(theta)*cos(psi),sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                   cos(theta)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                   -sin(theta)        ,sin(phi)*cos(theta)                           ,cos(phi)*cos(theta)                           ];
    vec_pointingVecToRgv_bluetooth = simParams.dcm_bluetooth2uas' * dcm_uas2enu' * vec_pointingVecToRgv_enu;

    % Get a specific vector that is orthogonal to the true
    % poining vector.
    vec_specificOrthogonal_bluetooth = [-vec_pointingVecToRgv_bluetooth(2);vec_pointingVecToRgv_bluetooth(1);0];
    % Generate a random rotation about the true pointing
    % vector.
    dcm_specificOrthogonal2randomOrthogonal = axang2rotm([vec_pointingVecToRgv_bluetooth',rand()*2*pi]);
    % Rotate the specific orthogonal vector by the random
    % rotation to get a random orthogonal vector. This is easy because the
    % specific orthogonal vector is already stored in a matrix.
    vec_randomOrthogonal_bluetooth = dcm_specificOrthogonal2randomOrthogonal*vec_specificOrthogonal_bluetooth;
    % Specify the axis and angle of rotation for the
    % estimated pointing vector. The axis is the random orthogonal vector
    % and the angle is random based on the pointing angle standard
    % deviation parameter.
    axang = [vec_randomOrthogonal_bluetooth',randn()*simParams.bluetoothAngleStdRad];
    % Generate the rotation matrix corresponding the the
    % specified axis and angle.
    dcm_truePointingVec2sensorPointingVec = axang2rotm(axang);
    % Rotate the true pointing vector by the pointing
    % error rotation to get the sensor pointing vector.
    vec_sensorPointingVec_bluetooth = dcm_truePointingVec2sensorPointingVec*vec_pointingVecToRgv_bluetooth;
end
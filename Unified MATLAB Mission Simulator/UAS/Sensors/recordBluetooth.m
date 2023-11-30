function vec_sensorPointingVec_bluetooth = recordBluetooth(simParams, uasState, rgvPosition)
    vec_pointingVecToRgv_enu = uasState(1:3)'-[rgvPosition;0];
    vec_pointingVecToRgv_enu = normalize3by1(vec_pointingVecToRgv_enu);
    vec_pointingVecToRgv_bluetooth = simParams.dcm_bluetooth2uas' * getDcmUas2Enu(uasState)' * vec_pointingVecToRgv_enu;

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
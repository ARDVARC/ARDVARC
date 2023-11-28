function trix_vec_sensorPointingVec_enu = getPinholePointingVecs(trix_vec_uas2rgv_enu, params)
    % Uses the pinhole camera model to get pointing vectors to the RGV from
    % true state information
    arguments(Input)
        trix_vec_uas2rgv_enu (:,3) double
        params (1,1) OrbAzParams
    end
    arguments(Output)
        trix_vec_sensorPointingVec_enu (:,3) double
    end
    
    N = size(trix_vec_uas2rgv_enu,1);

    trix_vec_truePointingVec_enu = trix_vec_uas2rgv_enu./vecnorm(trix_vec_uas2rgv_enu,2,2);
    trix_vec_cameraForwardVec_enu = getSensorPointingVecs(trix_vec_truePointingVec_enu, params);
    trix_vec_cameraLeftVecNOTNORMALIZED_enu = cross(repmat([0 0 1],[N 1]), trix_vec_cameraForwardVec_enu, 2);
    trix_vec_cameraLeftVec_enu = trix_vec_cameraLeftVecNOTNORMALIZED_enu./vecnorm(trix_vec_cameraLeftVecNOTNORMALIZED_enu,2,2);
    trix_vec_cameraUpVec_enu = cross(trix_vec_cameraForwardVec_enu, trix_vec_cameraLeftVec_enu, 2);
    trix3_dcm_cam2enu = [permute(trix_vec_cameraForwardVec_enu, [2,3,1]),permute(trix_vec_cameraLeftVec_enu, [2,3,1]),permute(trix_vec_cameraUpVec_enu, [2,3,1])];
    trix3_dcm_enu2cam = permute(trix3_dcm_cam2enu, [2,1,3]);

    trix_vec_uas2rgv_cam = permute(pagemtimes(trix3_dcm_enu2cam, permute(trix_vec_uas2rgv_enu, [2,3,1])), [3,1,2]);
    
    trix_vec_pixel = getPinholeCameraPixels(trix_vec_uas2rgv_cam, params.camParams);
    if (params.showPinholeFrames)
        showPinholeFrames(trix_vec_pixel, params.camParams);
    end
    trix_vec_sensorPointingVec_cam = pinholeCameraPixels2PointingVectors(trix_vec_pixel, params.camParams);

    trix_vec_sensorPointingVec_enu = permute(pagemtimes(trix3_dcm_cam2enu, permute(trix_vec_sensorPointingVec_cam, [2,3,1])), [3,1,2]);
end
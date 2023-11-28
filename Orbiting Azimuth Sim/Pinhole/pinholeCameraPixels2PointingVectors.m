function trix_vec_sensorPointingVec_cam = pinholeCameraPixels2PointingVectors(trix_vec_pixel, camParams)
    % Given a pinhole camera frame, calculates the camera-frame azimuth and
    % elevation of the white pixel based on camera parameters
    arguments(Input)
        trix_vec_pixel (:,2) double
        camParams (1,3) CamParams
    end
    arguments(Output)
        trix_vec_sensorPointingVec_cam (:,3) double
    end
    
    % set z coordinate to focal length
    z = camParams.focalLength;
    H = camParams.Height;
    W = camParams.Width;
    rho_w = camParams.rho_w;
    rho_h = camParams.rho_h;
    N = size(trix_vec_pixel,1);
    
    vec_x = (trix_vec_pixel(:,1) - W/2) * rho_w;
    vec_y = (trix_vec_pixel(:,2) - H/2) * rho_h;
    
    trix_vec_uas2rgvGuess_cam = [ones(N,1)*z,-vec_x,vec_y];
    trix_vec_sensorPointingVec_cam = trix_vec_uas2rgvGuess_cam./vecnorm(trix_vec_uas2rgvGuess_cam,2,2);
end

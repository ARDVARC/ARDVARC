function trix_vec_pixel = getPinholeCameraPixels(trix_vec_uas2rgv_cam, camParams)
    % Determines pinhole camera pixel locations given certain camera
    % parameters and a list of true RGV positions (relative to the camera)
    arguments(Input)
        trix_vec_uas2rgv_cam (:,3) double
        camParams (1,1) CamParams
    end

    H = camParams.Height;
    W = camParams.Width;
    rho_h = camParams.rho_h;
    rho_w = camParams.rho_w;
    f = camParams.focalLength;
    
    u_0 = W/2;
    v_0 = H/2;
    N = size(trix_vec_uas2rgv_cam,1);
    
    % extract the positions of the uas
    trix3_vec_uas2rgv_cam = permute(trix_vec_uas2rgv_cam, [3,2,1]);
    trix3_vec_X = trix3_vec_uas2rgv_cam(:,1,:);
    trix3_vec_Y = trix3_vec_uas2rgv_cam(:,2,:);
    trix3_vec_Z = trix3_vec_uas2rgv_cam(:,3,:);
    
    % Weird camera stuff
    K = [f/rho_w 0       u_0;
         0       f/rho_h v_0;
         0       0       1  ];
    P_0 = [1 0 0 0;
           0 1 0 0;
           0 0 1 0];
    xi_C = eye(4);
    
    % Weird camera premulitplicaiton matrix
    trix_cameraPremult = K * P_0 * xi_C;

    % Position stuff
    trix3_P_tilde = [-trix3_vec_Y;trix3_vec_Z;trix3_vec_X;ones(1,1,N)];
    
    % ???
    trix3_p_tilde = pagemtimes(trix_cameraPremult,trix3_P_tilde);

    trix3_vec_u_tilde = trix3_p_tilde(1,:,:);
    trix3_vec_v_tilde = trix3_p_tilde(2,:,:);
    trix3_vec_w_tilde = trix3_p_tilde(3,:,:);
    
    
    trix3_vec_u = floor(trix3_vec_u_tilde./trix3_vec_w_tilde);
    trix3_vec_v = floor(trix3_vec_v_tilde./trix3_vec_w_tilde);
    
    trix_vec_pixel = permute([trix3_vec_u,trix3_vec_v], [3,2,1]);
end

function showPinholeFrames(trix_vec_pixel, camParams)
    H = camParams.Height;
    W = camParams.Width;

    N = size(trix_vec_pixel,1);

    vec_u = trix_vec_pixel(:,1);
    vec_v = trix_vec_pixel(:,2);
    
    for i = 1:N
        u = vec_u(i);
        v = vec_v(i);
        img_array = zeros(H,W);
        
        % check if in FOV
        if v <= H && u <= W && v >= 0 && u >= 0
            img_array(v,u) = 1;
        end
        
        rendered_image = img_array;
        figure
        imshow(rendered_image)
    end
end
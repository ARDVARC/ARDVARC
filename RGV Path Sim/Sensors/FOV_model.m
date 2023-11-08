%% ARDVARC
% Models the fov's performance
theta_c = 15 * pi/180; %Camera mounting angle
vec_Height = linspace(10,60,6); %Height array to go through various heights

vec_theta_h = linspace((5*pi/180),(pi/2),100); %Iterative array for height FOV ranges
vec_theta_w = linspace(5*pi/180,120*pi/180,100); %Iterative array for width FOV ranges

trix_fovArea = cell(length(vec_Height),1);
vec_x_dist = zeros(length(vec_theta_h),1);
vec_y_dist = zeros(length(vec_theta_w),1);
vec_areas = zeros(length(vec_y_dist),1);
%Loop to iterate through results
for i = 1:length(vec_Height)
    for j = 1:length(vec_theta_h)
        %Loop through height
        vec_x_dist(j) = vec_Height(i) * (tan(theta_c + vec_theta_h(j)/2) - tan(theta_c - vec_theta_h(j)/2));
        for k = 1:length(vec_theta_w)
            vec_y_dist(k) = 2*vec_Height(i) * tan(vec_theta_w(k)/2)*tan(theta_c);
        end
    end
    trix_fovArea{i}(:,1) = vec_x_dist(:);
    trix_fovArea{i}(:,2) = vec_y_dist(:);
    trix_fovArea{i}(:,3) = vec_areas(:);  

    %Visualize the data
    figure(i)
    subplot(2,3,1)
end




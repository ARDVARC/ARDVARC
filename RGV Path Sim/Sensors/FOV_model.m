clc; close all; clear;
%% ARDVARC
% Models the fov's performance
theta_c = 15 * pi/180; %Camera mounting angle
vec_Height = linspace(10,60,6); %Height array to go through various heights

%Define the camera intrinsics
%Define the Raspberry pi HQ camera intrinsics (Rasberry Pi lense)
cam_rasp = cam;
cam_rasp.Height = 3040;
cam_rasp.Width = 4056;
cam_rasp.rho_h = 1.55e-6;
cam_rasp.rho_w = 1.55e-6;
vec_focal_length = linspace((0.001),(0.025),25); %Iterative array for height FOV ranges

%Speed based pre-allocations
trix_fovArea = cell(length(vec_Height),1);
vec_x_dist = zeros(length(vec_focal_length),1);
vec_y_dist = zeros(length(vec_focal_length),1);
vec_theta_h = zeros(length(vec_focal_length),1);
vec_theta_v = zeros(length(vec_focal_length),1);
vec_areas = zeros(length(vec_focal_length),1);

%Loop to iterate through results
for i = 1:length(vec_Height)
    for j = 1:length(vec_focal_length)
        %Loop through focal lengths and find corresponding fov angles
        vec_theta_h(j) = 2*atan2((cam_rasp.Width * cam_rasp.rho_w),(2*vec_focal_length(j)));
        vec_theta_v(j) = 2*atan2((cam_rasp.Height * cam_rasp.rho_h),(2*vec_focal_length(j)));
        %Loop through focal lengths and calculate area visualized
        vec_x_dist(j) = vec_Height(i) * (tan(theta_c + vec_theta_h(j)/2) - tan(theta_c - vec_theta_h(j)/2));
        vec_y_dist(j) = 2*vec_Height(i) * tan(vec_theta_v(j)/2)*tan(theta_c);
        
    end

    vec_areas = vec_x_dist .* vec_y_dist;
    %Store data
    trix_fovArea{i}(:,1) = vec_focal_length(:);
    trix_fovArea{i}(:,2) = vec_theta_h(:) * 180/pi;
    trix_fovArea{i}(:,3) = vec_theta_v(:) * 180/pi;  
    trix_fovArea{i}(:,4) = vec_x_dist(:);
    trix_fovArea{i}(:,5) = vec_y_dist(:);  
    trix_fovArea{i}(:,6) = vec_areas(:);
    
end

for i = 1:length(vec_Height)
    %Visualize the data
    figure(i)
    sgtitle(['Height = ',num2str(vec_Height(i))])
    subplot(2,3,1)
        plot(trix_fovArea{i}(:,1),trix_fovArea{i}(:,2)); hold on
        title('Focal Length vs. Theta_h')
        xlabel('Focal Length (m)')
        ylabel('Theta_h deg')

    subplot(2,3,2)
        plot(trix_fovArea{i}(:,1),trix_fovArea{i}(:,3)); hold on
        title('Focal Length vs. Theta_v')
        xlabel('Focal Length (m)')
        ylabel('Theta_v deg')

    subplot(2,3,3)
        plot(trix_fovArea{i}(:,1),trix_fovArea{i}(:,4)); hold on
        yline(150,'b--')
        title('Focal Length vs. Horizontal Distance seen')
        xlabel('Focal Length (m)')
        ylabel('X_{seen} ft')
        legend('','X limit of the viable boundary')

    subplot(2,3,4.5)
        plot(trix_fovArea{i}(:,1),trix_fovArea{i}(:,5)); hold on
        yline(150,'b--')
        title('Focal Length vs. Veritical Distance seen')
        xlabel('Focal Length (m)')
        ylabel('Y_{seen} ft')
        legend('','Y limit of the viable boundary')
    
    subplot(2,3,5.5)
        plot(trix_fovArea{i}(:,1),trix_fovArea{i}(:,6)); hold on
        yline(150^2,'b--')
        title('Focal Length vs. Area seen')
        xlabel('Focal Length (m)')
        ylabel('Area_{seen} ft^2')
        legend('','Area of the viable boundary')

end
function rendered_image = corkePinholeCamera(...
    rgv_vec,...
    camera)
%corkePinholeCamera See Corke Ch.11 for details 
% TODO(LF) put function stub here
% TODO(LF) BE CAREFUL OF FRAMES (what is rgv expressed in?)
% TODO(LF) Add all equation numbers
% TODO(LF) Model various resolutions

H = camera.Height;
W = camera.Width;

u_0 = W/2;
v_0 = H/2;

% TODO(LF) change this later in a way that makes sense
rho_h = camera.rho_h;
rho_w = camera.rho_w;

% TODO(LF) make this an argument you pass in
% camera pose
pose_rot = eye(3);
pose_vec = [0;0;0];

% TODO(LF) Pull this from the camera object
% Extrinsics Matrix
xi_C = [pose_rot pose_vec;
        0 0 0 1]; 

% xi_C = 1e3 * [1.5 0 .64 0;...
%               0 1.5 .512 0;...
%               0 0 .0010 0];

% Going to try to match notation to the book as closely as possible
f = camera.focalLength;

% extrace the positions of the uas
X = rgv_vec(1);
Y = rgv_vec(2);
Z = rgv_vec(3);

% homogenouse etc.
P_tilde = [X;Y;Z;1];

% homogenouse etc.
P_0 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0];

% camera intrinsics matrix
K = [f/rho_w 0 u_0;
           0 f/rho_h v_0;
           0 0 1];

p_tilde = K * P_0 * xi_C * P_tilde;

u_tilde = p_tilde(1)/W;
v_tilde = p_tilde(2)/H;
w_tilde = p_tilde(3);


u = floor(u_tilde/w_tilde);
v = floor(v_tilde/w_tilde);

img_array = zeros(H,W);

% check if in FOV
if v <= H && u <= W
    img_array(v,u) = 1;
end

rendered_image = img_array;
imshow(rendered_image)


end

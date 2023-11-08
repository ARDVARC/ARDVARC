function [azimuth, elevation] = frame2AzAndEle(...
    frame,...
    cam)
% frame2AzAndEle
% TODO(LF)
%   C = ADDME(A) adds A to itself.
%
%   C = ADDME(A,B) adds A and B together.
%   Args:
%       - frame : a picture frame with centroid marked or something
%       - f : focal length of camera that took the frame [m]
%   See also SUM, PLUS.

% set z coordinate to focal length
z = cam.focalLength;
H = cam.Height;
W = cam.Width;
rho_w = cam.rho_w;
rho_h = cam.rho_h;
% row and col of the "centroid" of the tracked object
[row, col] = find(frame);

v = row;
u = col;

x = (u - W/2) * rho_w;
y = (v - H/2) * rho_h;

% azimuth = atan(x/z);
% elevation = atan(y/z);

azimuth = atan2(x,z);
elevation = -1 * atan2(y,z);



end


cam1 = cam
cam1.focalLength = .002;
cam1.Height = 200;
cam1.Width = 400;

cam1.rho_h = 1.4e-6;
cam1.rho_w = 1.4e-6;

frame = corkePinholeCamera([1.3,0,10],...
                            cam1);

[az, ele] = frame2AzAndEle(frame,cam1);

function [RGVapproxPos,UAVpos] = orbitDetermination(RGVabsPos,numPoints,orbRad,errorRad,UAValt)
%% Function
%         Create a predicted flight path the UAV will use during fine
%         localization
%% Inputs
%         RGVabsPos: Absolute location of RGV
%         numPoints: Number of data points along 1 orbit revolution
%         orbRad: Radius of UAV orbit
%         errorRad: Radius of error around absolute RGV location
%         UAValt: Altitude of UAV
%% --------------------------------------------------------------------

%% Create RGV Approximate Location
          % Random x
          x = (RGVabsPos(1)-errorRad) + ((RGVabsPos(1)+errorRad)-(RGVabsPos(1)-errorRad))*rand(1,1);
          % Random y
          y = (RGVabsPos(2)-errorRad) + ((RGVabsPos(2)+errorRad)-(RGVabsPos(2)-errorRad))*rand(1,1);
          % Z position does not change
          z = RGVabsPos(3);
          % Create position vector
          RGVapproxPos = [x,y,z]; %[m]

%% Create Orbit Path
          %Step Size
          d = 2*pi*orbRad/numPoints; %[m]
          thetaStep = d/orbRad; %[rad]
          %Starting point
          UAVpos(1,:) = [RGVapproxPos(1)+orbRad,RGVapproxPos(2),UAValt]; %[m]

          %Loop through steps
          theta = thetaStep;
          for i = 2:numPoints
              UAVpos(i,1) = orbRad*cos(theta) + RGVapproxPos(1);
              UAVpos(i,2) = orbRad*sin(theta) + RGVapproxPos(2);
              UAVpos(i,3) = UAValt;
              theta = theta+thetaStep;
          end

%% Fix for BLE Sim
          %Make it a vector
          for i = 1:numPoints
              temp_RGVapproxPos(i,:) = RGVapproxPos();
          end

          UAVpos = UAVpos';
          RGVapproxPos = temp_RGVapproxPos';

end


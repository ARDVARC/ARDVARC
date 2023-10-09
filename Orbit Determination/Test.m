%% Description:
%         Test your function here
%% Modification History
%         10/09/23 -> Zachary Z, orbitPath function call
%% -----------------------------------------

clc, clear

%% Set Constants
          % orbitDetermination inputs
          RGVabsPos = [5,5,0]; %[m]
          numPoints = 50;
          orbRad = 1; %[m]
          errorRad = 1; %[m]
          UAValt = 18; %[m]
          
%% Call Functions
[RGVpos,orbitPath] = orbitDetermination(RGVabsPos,numPoints,orbRad,errorRad,UAValt);

%% Plot Values
i = (1:50);
figure(1)
hold on
    plot3(RGVabsPos(1),RGVabsPos(2),RGVabsPos(3),'.')
    plot3(RGVpos(1),RGVpos(2),RGVpos(3),'x');
    plot3(orbitPath(i,1),orbitPath(i,2),orbitPath(i,3),'-o');
hold off
legend("True RGV","Approx RGV","UAS Orbit")
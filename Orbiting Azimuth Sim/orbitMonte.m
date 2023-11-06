clc
clear
close all

minDist = 1;
maxDist = 23;
distGap = 0.02;

distances = minDist:distGap:maxDist;
N = length(distances);
errors1deg = zeros(1,N);
errors3deg = zeros(1,N);
errors5deg = zeros(1,N);

tic
for i = 1:N
    errors1deg(i) = tryOrbit(distances(i), 1);
    errors3deg(i) = tryOrbit(distances(i), 3);
    errors5deg(i) = tryOrbit(distances(i), 5);
end
toc

figure
hold on
grid minor
axis equal
scatter(distances,errors1deg,MarkerFaceColor="#0072BD",Marker=".",DisplayName="1 Degree STD")
scatter(distances,errors3deg,MarkerFaceColor="#D95319",Marker=".",DisplayName="3 Degree STD")
scatter(distances,errors5deg,MarkerFaceColor="#EDB120",Marker=".",DisplayName="5 Degree STD")
yline(2, 'k:', DisplayName="Coarse Localization Target")
yline(1, 'r:', DisplayName="Fine Localization Target")
xlabel("Orbit Distance [m]")
ylabel("Estimate Error [m]")
title("RGV Position Estimate Error for Different Orbital Ground Distances")
xlim([minDist maxDist])
ylim([0 max([errors5deg, errors3deg, errors3deg], [], "all")])
legend(Location="best")

function error = tryOrbit(orbitDistance, angleStdDeg)
    seed = randi(1000000);
    % orbitDistance = 1000;
    duration = 30;
    orbitSpeed = 2*pi*orbitDistance/duration;
    sampleRate = 1;
    height = 10;
    heightStd = 0;
    angleStd = deg2rad(angleStdDeg);
    zoomOnBottom = true;
    
    rng(seed);
    orbitAngularSpeed = orbitSpeed / orbitDistance;
    angleBetweenSamples = orbitAngularSpeed / sampleRate;
    sampleCount = floor(duration*sampleRate) + 1;
    sampleAngles = 0:angleBetweenSamples:((sampleCount-1)*angleBetweenSamples);
    sampleHeights = randn(1,sampleCount)*heightStd+height;
    samplePositions = [orbitDistance*sin(sampleAngles)',orbitDistance*cos(sampleAngles)',sampleHeights'];
    pointingVecs = -samplePositions./vecnorm(samplePositions,2,2);
    
    
    orthogonalVectorsBig = zeros(3, 3, sampleCount);
    orthogonalVectorsBig(:,1,:) = [-pointingVecs(:,2) pointingVecs(:,1) zeros(sampleCount,1)]';
    randomRotationsAboutPointingVectors = axang2rotm([pointingVecs rand(sampleCount, 1)*2*pi]);
    randomOrthogonalVectorsBig = pagemtimes(randomRotationsAboutPointingVectors, orthogonalVectorsBig);
    randomOrthogonalVectors = reshape(randomOrthogonalVectorsBig(:,1,:), 3, sampleCount)';
    axangs = [randomOrthogonalVectors randn(sampleCount,1)*angleStd];
    rotms = axang2rotm(axangs);
    pointingVectorsBig = zeros(3, 3, sampleCount);
    pointingVectorsBig(:,1,:) = pointingVecs';
    rotatedPointingVectorsBig = pagemtimes(rotms, pointingVectorsBig);
    rotatedPointingVectors = reshape(rotatedPointingVectorsBig(:,1,:), 3, sampleCount)';
    
    % opt = optimset(TolFun=1e-10,TolX=1e-10);
    opt = optimset();
    bestGuess = fminsearch(@(x) sqrt(sum(vecnorm(cross(rotatedPointingVectors,x-samplePositions,2),2,2).^2))/sampleCount, [0,0,0], opt);
    error = norm(bestGuess);
    % bestGuess2 = fsolve(@(x) sum(vecnorm(cross(rotatedPointingVectors,x-samplePositions,2),2,2))/sampleCount, [0,0,0], opt);
    % bestGuess3 = fsolve(@(x) (sum(vecnorm(cross(rotatedPointingVectors,x-samplePositions,2),2,2).^3))^(1/3)/sampleCount, [0,0,0], opt);
    
    % figure
    % scatter3(samplePositions(:,1),samplePositions(:,2),samplePositions(:,3))
    % hold on
    % grid minor
    % axis equal
    % axis vis3d
    % scatter3(0,0,0,'g','filled','square')
    % scatter3(bestGuess(1),bestGuess(2),bestGuess(3),'r','filled','square')
    % % scatter3(bestGuess2(1),bestGuess2(2),bestGuess2(3),'k','filled','square')
    % % scatter3(bestGuess3(1),bestGuess3(2),bestGuess3(3),'m','filled','square')
    % setAxes3DPanAndZoomStyle(zoom(gca),gca,'camera')
    % if (zoomOnBottom)
    %     xlim([-orbitDistance/4 orbitDistance/4])
    %     ylim([-orbitDistance/4 orbitDistance/4])
    %     zlim([-orbitDistance/4 orbitDistance/2])
    % else
    %     xlim([-orbitDistance orbitDistance])
    %     ylim([-orbitDistance orbitDistance])
    %     zlim([0 height + 3*heightStd])
    % end
    % for i = 1:sampleCount
    %     plot3([samplePositions(i,1),samplePositions(i,1)+pointingVecs(i,1)],[samplePositions(i,2),samplePositions(i,2)+pointingVecs(i,2)],[samplePositions(i,3),samplePositions(i,3)+pointingVecs(i,3)], 'g')
    %     plot3([samplePositions(i,1) samplePositions(i,1)+pointingVecs(i,1)*2*height], [samplePositions(i,2) samplePositions(i,2)+pointingVecs(i,2)*2*height], [samplePositions(i,3) samplePositions(i,3)+pointingVecs(i,3)*2*height], 'g:')
    %     plot3([samplePositions(i,1),samplePositions(i,1)+rotatedPointingVectors(i,1)],[samplePositions(i,2),samplePositions(i,2)+rotatedPointingVectors(i,2)],[samplePositions(i,3),samplePositions(i,3)+rotatedPointingVectors(i,3)], 'r')
    %     plot3([samplePositions(i,1) samplePositions(i,1)+rotatedPointingVectors(i,1)*2*height], [samplePositions(i,2) samplePositions(i,2)+rotatedPointingVectors(i,2)*2*height], [samplePositions(i,3) samplePositions(i,3)+rotatedPointingVectors(i,3)*2*height], 'r:')
    % end
end
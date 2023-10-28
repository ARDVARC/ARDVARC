clc
clear
close all

runCount = 100;
idealJointDuration = 20;
plotRate = 10;
duration = 30*60;

bests = zeros(runCount,1);
for i = 1:runCount
    [times, rgv1positions, rgv2positions] = simulateTwoRGVs(plotRate, duration, randi(10000), randi(10000));
    distanceBetweenRGVs = vecnorm(rgv1positions - rgv2positions, 2, 2);
    movingMaxForJoint = movmax(distanceBetweenRGVs, idealJointDuration*plotRate);
    bests(i) = min(movingMaxForJoint);
    fprintf("Finished run %i of %i\n", i, runCount)
end
histogram(bests, 0:0.5:(max(bests)+0.5))
xlabel("Best Sustained Distance [m]")
ylabel("Frequency")
grid minor
title(sprintf("Joint Localization Best Sustained Distances Across %i Simulations", runCount))
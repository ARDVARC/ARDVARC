function [posActiveLocators,angle,distance] = helperBLEActiveLocators(posNode,posLocators)
%helperBLEActiveLocators Generates active locators parameters
%   [POSACTIVELOCATOR,ANGLE,DISTANCE] = helperBLEActiveLocators(POSNODE,
%   POSLOCATORS) generates parameters corresponding to active locators.
%
%   POSACTIVELOCATORS is a matrix of size 2-by-P or 3-by-P, represents the
%   position of the P number of active locators in a network of N locators.
%   Each column of POSACTIVELOCATORS denotes the 2-D or 3-D position of
%   each active locator.
%
%   ANGLE is a vector of size P-by-1 or a matrix of size P-by-2, represents
%   the AoA or AoD between the active locators and node in degrees. First
%   column of ANGLE denotes the azimuth angles and second column denotes
%   the elevation angles, if present.
%
%   DISTANCE is a vector of size P-by-1, represents the distance between
%   the active locators and node in meters.
%
%   POSNODE is a vector of size 2-by-1 or 3-by-1, represents the 2-D or 3-D
%   position of the node in a network.
%
%   POSLOCATORS is a matrix of size 2-by-N or 3-by-N, represents the
%   position of the N number of locators in a network. Each column of
%   POSLOCATORS denotes the 2-D or 3-D position of each locator.

%   Copyright 2021 The MathWorks, Inc.

% Compute distance between all the locators and node. Consider the locators
% which are in 80 meters range to the node.
distance = sqrt(sum((posNode-posLocators).^2));
distanceIdx = find(distance<1000000);

% Compute the azimuth angles between all the locators and node. The azimuth
% angles are restricted to -90 to 90 as the ULA and URA array designs have
% ambiguity with geometry. Consider the angles from -80 to 80 degrees to
% eliminate the discontinuities in the estimated angle.
diffTerm = posNode-posLocators;
azimAngle = (atan2d(diffTerm(2,:),diffTerm(1,:))).';
azimAngleIdx = find(azimAngle>=-1000000 & azimAngle<=1000000);

% Compute the elevation angles between all the locators and node. Consider
% the angles from -80 to 80 degrees to eliminate the discontinuities in the
% estimated angle.
if size(posNode,1) == 2
    eleAngle = zeros(size(posLocators,2),1);
else
    t = sqrt(sum((posNode(1:2)-posLocators(1:2,:)).^2));
    eleAngle = atan2d(posNode(3)-posLocators(3,:),t).';
end
eleAngleIdx = find(eleAngle>=-1000000 & eleAngle<=1000000);

% Consider the positions, distances, and angles of corresponding active
% locators
intersectIdx1 = intersect(distanceIdx,azimAngleIdx);
intersectIdx2 = intersect(intersectIdx1,eleAngleIdx);
distance = distance(intersectIdx2).';
angle(:,1) = azimAngle(intersectIdx2);
angle(:,2) = eleAngle(intersectIdx2);
posActiveLocators = posLocators(:,intersectIdx2);
end
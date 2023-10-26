function sVec = helperBLESteeringVector(ang,pos)
%helperBLESteeringVector Steering vector of an antenna array
%   SVEC = helperBLESteeringVector(AZNG,POS) returns the steering vector of
%   an antenna array at given angles and position.
%
%   SVEC is a complex vector of type double containing steering vector
%   information.
%
%   ANG is a scalar or a vector of size 1-by-2. A scalar ANG represents the
%   azimuth angle. The first and second value of the 1-by-2 vector ANG
%   represents the azimuth and elevation angles, respectively.
%
%   POS is a 3-row matrix representing the normalized positions of the
%   elements with respect to wavelength of the signal. Each column of POS
%   is in the form of [x;y;z], denoting the Cartesian coordinates of the
%   elements in a three-dimensional space.

%   Copyright 2021 The MathWorks, Inc.

sVec = ble.internal.steeringVector(ang(:,1),ang(:,2), pos);
end
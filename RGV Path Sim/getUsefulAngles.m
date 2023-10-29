function [rollError, pitchError, yawError, toNearRGVB] = getUsefulAngles(state, RB2E, RGVxyz, targetHeight)
    targetRGVgroundDistance = 3;

    xyz = state(1:3);
    RE2B = RB2E^-1;
    
    headingB = [1;0;0];
    headingE = RB2E*headingB;

    toAboveRGVE = RGVxyz - xyz + [0;0;-targetHeight];
    flatToAboveRGVE = normalize(toAboveRGVE(1:2));
    toNearRGVE = toAboveRGVE - targetRGVgroundDistance*[flatToAboveRGVE;0];

    desiredHeadingE = normalize(toNearRGVE);
    toNearRGVB = RE2B*toNearRGVE;
    rightB = [0;1;0];
    rightE = RB2E*rightB;
    flatHeadingE = normalize(headingE(1:2));
    flatDesiredHeadingE = normalize(desiredHeadingE(1:2));

    pitchError = asin(headingE(3));
    yawError = signedAngle(flatDesiredHeadingE, flatHeadingE);
    rollError = -asin(rightE(3));
end
function ang = signedAngle(u, v)
    % Calculates the angle from 2D vector 'u' to 2D vector 'v'
    % (counter-clockwise is positive)
    arguments(Input)
        u (1,2) double
        v (1,2) double
    end
    arguments(Output)
        ang (1,1) double
    end
    ang = atan2(v(2)*u(1)-v(1)*u(2),v(1)*u(1)+v(2)*u(2));
end
function ang = signedAngle(u, v)
    ang = atan2(v(2)*u(1)-v(1)*u(2),v(1)*u(1)-v(2)*u(2));
end
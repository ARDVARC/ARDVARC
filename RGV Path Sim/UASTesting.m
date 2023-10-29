clc
clear
close all

RGVxyz = [50;50;0];

[ts, states] = ode45(@(t,state) EOM(t, state, RGVxyz), [0 500], [0;0;-10;0;0;0;0;0;0;0;0;0]);

figure
view([-1;-1;1])
hold on
axis equal
grid minor
xlabel("X")
ylabel("Y")
zlabel("Z")
plot3(states(:,1),states(:,2),-states(:,3))
scatter3(RGVxyz(1),RGVxyz(2),-RGVxyz(3))

figure
hold on
grid minor
xlabel("Time")
ylabel("State")
pitchError = zeros(1,length(ts));
yawError = zeros(1,length(ts));
rollError = zeros(1,length(ts));
toNearRGVB = zeros(3,length(ts));
L = zeros(1,length(ts));
M = zeros(1,length(ts));
N = zeros(1,length(ts));
Z = zeros(1,length(ts));
for i = 1:length(ts)
    [L(i),M(i),N(i),Z(i)] = getLMNZ(ts(i), states(i,:)', getRB2E(states(i,:)'), RGVxyz);
    [rollError(i), pitchError(i), yawError(i), toNearRGVB(:,i)] = getUsefulAngles(states(i,:)', getRB2E(states(i,:)), RGVxyz, 10);
end
plot(ts, rollError, DisplayName="rollError")
plot(ts, pitchError, DisplayName="pitchError")
plot(ts, yawError, DisplayName="yawError")
% plot(ts, states(:,1), DisplayName="X")
% plot(ts, states(:,2), DisplayName="Y")
% plot(ts, -states(:,3), DisplayName="Z")
% plot(ts, states(:,4), DisplayName="phi")
% plot(ts, states(:,5), DisplayName="theta")
% plot(ts, states(:,6), DisplayName="psi")
% plot(ts, states(:,7), DisplayName="u")
% plot(ts, states(:,8), DisplayName="v")
% plot(ts, -states(:,9), DisplayName="w")
% plot(ts, states(:,10), DisplayName="p")
% plot(ts, states(:,11), DisplayName="q")
% plot(ts, states(:,12), DisplayName="r")
% plot(ts, L, DisplayName="L")
% plot(ts, M, DisplayName="M")
% plot(ts, N, DisplayName="N")
% plot(ts, Z, DisplayName="Z")
% plot(ts, toNearRGVB(1,:), DisplayName="toNearRGVBX")
% plot(ts, toNearRGVB(2,:), DisplayName="toNearRGVBY")
% plot(ts, toNearRGVB(3,:), DisplayName="toNearRGVBZ")
legend

% figure
% hold on
% grid minor
% xlabel("Time")
% ylabel("Meters")
% legend

% figure
% hold on
% grid minor
% xlabel("Time")
% ylabel("State")
% plot(ts, rollError'-states(:,4), DisplayName="rolldiff")
% plot(ts, pitchError'-states(:,5), DisplayName="pitchdiff")
% plot(ts, yawError'-states(:,6), DisplayName="yawdiff")
% legend

function dState = EOM(t, state, RGVxyz)
    m = 1;
    Ix = 1;
    Iy = 1;
    Iz = 1;

    RGVxyz = eul2rotm([t*2*pi/157,0,0])*RGVxyz;
    
    phi = state(4);
    theta = state(5);
    psi = state(6);
    uvw = state(7:9);
    p = state(10);
    q = state(11);
    r = state(12);
    pqr = state(10:12);

    RB2E = getRB2E(state);
    rotDynMat = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);
                 0,cos(phi)           ,-sin(phi)          ;
                 0,sin(phi)*sec(theta),cos(phi)*sec(theta)];

    [Lc,Mc,Nc,Zc] = getLMNZ(t, state, RB2E, RGVxyz);
    xyzdot = RB2E*uvw;

    phithetapsidot = rotDynMat*pqr;

    gravDir = [-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)];
    % TODO - Aerodynamic forces
    uvwdot = cross(uvw,pqr) + 9.81*gravDir + [0;0;Zc]/m;
    
    % TODO - Aerodynamic moments
    pqrdot = [(Iy-Iz)/Ix*q*r;(Iz-Ix)/Iy*p*r;(Ix-Iy)/Iz*p*q] + [Lc/Ix;Mc/Iy;Nc/Iz];

    dState = [xyzdot;phithetapsidot;uvwdot;pqrdot];
end

function [L,M,N,Z] = getLMNZ(t, state, RB2E, RGVxyz)
    targetHeight = 10;
    LMNmax = 10;
    Zmax = 50;
    
    z = state(3);
    u = state(7);
    v = state(8);
    w = state(9);
    wE = norm(RB2E*[0;0;w]);
    p = state(10);
    q = state(11);
    r = state(12);

    [rollError, pitchError, yawError, toNearRGVB] = getUsefulAngles(state, RB2E, RGVxyz, targetHeight);

    % L = 400*rollError-80*v-200*p+140*clamp(toNearRGVB(2),-5,5);
    L = 800*rollError-80*v-200*p+140*clamp(toNearRGVB(2),-5,5);
    % L = 0;
    % M = 400*pitchError+80*u-200*q-140*clamp(toNearRGVB(1),-5,5);
    M = 800*pitchError+80*u-200*q-140*clamp(toNearRGVB(1),-5,5);
    % M = 0;
    N = -1200*yawError-200*r;
    % Z = -9.81/cos(pitchError)/cos(rollError)+2*clamp(toNearRGVB(3),-5,5)-4*clamp(w,-1,1);
    Z = -9.81/cos(pitchError)/cos(rollError)+2*-clamp(z+targetHeight,-5,5)-4*clamp(w,-1,1);

    L = clamp(L,-LMNmax,LMNmax);
    M = clamp(M,-LMNmax,LMNmax);
    N = clamp(N,-LMNmax,LMNmax);
    Z = clamp(Z,-Zmax,Zmax);
end

function val = clamp(val, l, u)
    val = min(max(l, val), u);
end
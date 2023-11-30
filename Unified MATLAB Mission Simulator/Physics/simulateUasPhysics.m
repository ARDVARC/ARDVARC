function [vec_time, trix_vec_trueUasState] = simulateUasPhysics(simParams, vec_goToCenter_enu, vec_lookAt_enu, orbitRadius, startTime, endTime, vec_trueUasStartState)
    % Uses ode45 to simulate the physics of the UAS over a specified time
    % period and under specified initial conditions
    arguments(Input)
        simParams (1,1) SimParams
        vec_goToCenter_enu (3,1) double
        vec_lookAt_enu (2,1) double
        orbitRadius (1,1) double
        startTime (1,1) double
        endTime (1,1) double
        vec_trueUasStartState (12,1) double
    end
    opts = odeset("AbsTol",1e-6);
    [vec_time, trix_vec_trueUasState] = ode45(@(t,vec_trueUasState) ode45Inner(t, vec_trueUasState, simParams, vec_goToCenter_enu, vec_lookAt_enu, orbitRadius), [startTime endTime], vec_trueUasStartState, opts);

    function vec_dTrueUasState = ode45Inner(t, vec_trueUasState, simParams, vec_goToCenter_enu, vec_lookAt_enu, orbitRadius)
        % Use PD control to determine what forces and moments will get us
        % from where we think we are to where we want to be
        vec_goToReal_enu = vec_goToCenter_enu + orbitRadius*[-cos(2*pi*t/simParams.orbitDuration);sin(2*pi*t/simParams.orbitDuration);0];
        [Lc,Mc,Nc,Zc] = getMomentsAndZForce(simParams, vec_trueUasState, vec_goToReal_enu, vec_lookAt_enu);

        % Do actual physics
        vec_dTrueUasState = equationsOfMotion(simParams, vec_trueUasState, Lc, Mc, Nc, Zc);
    end
end
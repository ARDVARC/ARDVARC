clc
clear
close all

% INPUTS
d = 0.1;
noise_rad = deg2rad(0.01);

vu = [0 0 10];
v1_true = [10 1 0];
N = 1000;

% SOLVE
v2_true = v1_true + [0 d 0];
v3_true = v1_true + [d 0 0];
v4_true = v1_true + [d d 0];
v_center_true = (v1_true + v2_true + v3_true + v4_true)./4;
env_true = get_true_env_from_givens(v1_true, v2_true, v3_true, v4_true, vu);
choice = listdlg(ListString=["Plot Exact", "Simulate with Error"], SelectionMode='single', PromptString='Choose Method');

if (choice == 1)
    plot_no_error(env_true)
elseif (choice == 2)
    res = zeros(N,16);
    mean_error = zeros(1,N);
    centers = zeros(N,3);
    center_error = zeros(1,N);
    mean_noise = zeros(1,N);
    for i = 1:N
        env = env_true;
        noise = (rand(1,4) - 0.5)*2*noise_rad;
        mean_noise(i) = mean(abs(noise));
        n1 = [-env.v1p(2) env.v1p(1) 0];
        n1 = (axang2rotm([env.v1p rand(1)*2*pi]) * n1')';
        rotm1 = axang2rotm([n1 noise(1)]);
        env.v1p = (rotm1 * env.v1p')';

        n2 = [-env.v2p(2) env.v2p(1) 0];
        n2 = (axang2rotm([env.v2p rand(1)*2*pi]) * n2')';
        rotm2 = axang2rotm([n2 noise(2)]);
        env.v2p = (rotm2 * env.v2p')';

        n3 = [-env.v3p(2) env.v3p(1) 0];
        n3 = (axang2rotm([env.v3p rand(1)*2*pi]) * n3')';
        rotm3 = axang2rotm([n3 noise(3)]);
        env.v3p = (rotm3 * env.v3p')';

        n4 = [-env.v4p(2) env.v4p(1) 0];
        n4 = (axang2rotm([env.v4p rand(1)*2*pi]) * n4')';
        rotm4 = axang2rotm([n4 noise(4)]);
        env.v4p = (rotm4 * env.v4p')';

        res(i,:) = simulate_once(env);
        
        centers(i,:) = (res(i,1:3) + res(i,4:6) + res(i,7:9) + res(i,10:12))./4;
        center_error(i) = norm(centers(i,:) - v_center_true);
        
        error_v1 = norm(v1_true-res(i,1:3));
        error_v2 = norm(v2_true-res(i,4:6));
        error_v3 = norm(v3_true-res(i,7:9));
        error_v4 = norm(v4_true-res(i,10:12));
        mean_error(i) = mean([error_v1, error_v2, error_v3, error_v4]);
        
        % plot_no_error(1, env);
    end
    figure
    hold on
    grid minor
    scatter(rad2deg(mean_noise), mean_error, '.k')
    xlabel("Mean Pointing Vector Angle Error [deg]")
    ylabel("Mean Emitter Position Error [m]")
    xlim([0 rad2deg(noise_rad)])
    coeffs = polyfit(rad2deg(mean_noise), mean_error, 1);
    a = coeffs(1);
    b = coeffs(2);
    fplot(@(x) a*x+b, 'r:')
    disp("Mean Emitter Position Error: " + mean(mean_error))
    figure
    hold on
    grid minor
    scatter(rad2deg(mean_noise), center_error, '.k')
    xlabel("Mean Pointing Vector Angle Error [deg]")
    ylabel("Emitter Position Center Error [m]")
    xlim([0 rad2deg(noise_rad)])
    coeffs = polyfit(rad2deg(mean_noise), center_error, 1);
    a = coeffs(1);
    b = coeffs(2);
    fplot(@(x) a*x+b, 'r:')
    disp("Mean Center Error: " + mean(center_error))
end
%%
function make_me_zero = fsolve_me(x, env)
    v1 = x(1:3);
    v2 = x(4:6);
    v3 = x(7:9);
    v4 = x(10:12);
    lambda_1 = x(13);
    lambda_2 = x(14);
    lambda_3 = x(15);
    lambda_4 = x(16);
    
    make_me_zero = zeros(1,20);

    make_me_zero(1:3) = env.vu+lambda_1*env.v1p-v1;
    make_me_zero(4:6) = env.vu+lambda_2*env.v2p-v2;
    make_me_zero(7:9) = env.vu+lambda_3*env.v3p-v3;
    make_me_zero(10:12) = env.vu+lambda_4*env.v4p-v4;
    make_me_zero(13) = norm(v1-v2)-env.d12;
    make_me_zero(14) = norm(v1-v3)-env.d13;
    make_me_zero(15) = norm(v1-v4)-env.d14;
    make_me_zero(16) = norm(v2-v3)-env.d23;
    make_me_zero(17) = norm(v2-v4)-env.d24;
    make_me_zero(18) = norm(v3-v4)-env.d34;
end

function res = simulate_once(env)
    % SOLVE
    options = optimset('Display','off','Algorithm','levenberg-marquardt');
    res = fsolve(@(x) fsolve_me(x, env), zeros(1, 16), options);
    
    % CORRECT FOR BACKWARDS-NESS
    if (res(13) < 0)
        % It went backwards, flip it
        res(13:15) = -res(13:15);
        res(1:3) = -res(1:3) + 2 * env.vu;
        res(4:6) = -res(4:6) + 2 * env.vu;
        res(7:9) = -res(7:9) + 2 * env.vu;
        res(10:12) = -res(10:12) + 2 * env.vu;
    end
end

function plot_no_error(fignum, env)
    % SOLVE
    res = simulate_once(env);
    
    % PLOT IN 3D
    figure(fignum)
    scatter3(env.vu(1), env.vu(2), env.vu(3), 'filled', 'r')
    hold on
    grid minor
    axis equal
    plot3([env.vu(1), env.vu(1) + env.v1p(1)], [env.vu(2), env.vu(2) + env.v1p(2)], [env.vu(3), env.vu(3) + env.v1p(3)], 'k')
    plot3([env.vu(1), env.vu(1) + env.v2p(1)], [env.vu(2), env.vu(2) + env.v2p(2)], [env.vu(3), env.vu(3) + env.v2p(3)], 'k')
    plot3([env.vu(1), env.vu(1) + env.v3p(1)], [env.vu(2), env.vu(2) + env.v3p(2)], [env.vu(3), env.vu(3) + env.v3p(3)], 'k')
    plot3([env.vu(1), env.vu(1) + env.v4p(1)], [env.vu(2), env.vu(2) + env.v4p(2)], [env.vu(3), env.vu(3) + env.v4p(3)], 'k')
    scatter3(res(1), res(2), res(3), 'filled', 'r')
    scatter3(res(4), res(5), res(6), 'filled', 'g')
    scatter3(res(7), res(8), res(9), 'filled', 'b')
    scatter3(res(10), res(11), res(12), 'filled', 'y')
    plot3([res(1), res(4)],[res(2), res(5)], [res(3), res(6)], 'k')
    plot3([res(4), res(7)],[res(5), res(8)], [res(6), res(9)], 'k')
    plot3([res(7), res(10)],[res(8), res(11)], [res(9), res(12)], 'k')
    plot3([res(10), res(1)],[res(11), res(2)], [res(12), res(3)], 'k')
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
end

function env = get_true_env_from_givens(v1_true, v2_true, v3_true, v4_true, vu)
    env.vu = vu;
    env.v1p = v1_true - env.vu;
    env.v1p = env.v1p / norm(env.v1p);
    env.v2p = v2_true - env.vu;
    env.v2p = env.v2p / norm(env.v2p);
    env.v3p = v3_true - env.vu;
    env.v3p = env.v3p / norm(env.v3p);
    env.v4p = v4_true - env.vu;
    env.v4p = env.v4p / norm(env.v4p);
    env.d12 = norm(v1_true - v2_true);
    env.d13 = norm(v1_true - v3_true);
    env.d14 = norm(v1_true - v4_true);
    env.d23 = norm(v2_true - v3_true);
    env.d24 = norm(v2_true - v4_true);
    env.d34 = norm(v3_true - v4_true);
end
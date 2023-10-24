clc
clear
close all

% INPUTS
d = 0.1;
noise_deg = 0.01;

uas_ = [0 0 10];
rgv_ = [10 1 0];
emitter_count = 4;

N = 1000;

% PREP
emitter_angles = linspace(0,2*pi,emitter_count)';
emitters__ = rgv_ + d*[sin(emitter_angles) cos(emitter_angles) zeros(emitter_count,1)];
knowns_true = knowns.get_knowns_from(uas_,emitters__);

res = zeros(N,emitter_count*3);
mean_error = zeros(1,N);
centers = zeros(N,3);
center_error = zeros(1,N);
mean_noise = zeros(1,N);
for i = 1:N
    [knowns_predicted, mean_noise(i)] = knowns_true.shake(noise_deg);

    emitters_predicted__ = simulate_once(knowns_predicted);
    res(i,:) = emitters_predicted__(:);
    centers(i,:) = sum(emitters_predicted__,1)/emitter_count;
    center_error(i) = norm(centers(i,:) - rgv_);
    mean_error(i) = mean(vecnorm(emitters_predicted__ - emitters__, 2, 2));
    
    % plot_no_error(1, env);
end
figure
hold on
grid minor
scatter(rad2deg(mean_noise), mean_error, '.k')
xlabel("Mean Pointing Vector Angle Error [deg]")
ylabel("Mean Emitter Position Error [m]")
xlim([0 noise_deg])
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
xlim([0 noise_deg])
coeffs = polyfit(rad2deg(mean_noise), center_error, 1);
a = coeffs(1);
b = coeffs(2);
fplot(@(x) a*x+b, 'r:')
disp("Mean Center Error: " + mean(center_error))

% plot_no_error(10, knowns_true, emitters__)

%%
function make_me_zero = fsolve_me(x, k)
    arguments
        x double
        k knowns
    end
    emitters_predicted__ = reshape(x(1:k.emitter_count*3), [], 3);
    lambdas_ = x(k.emitter_count*3+1:end);
    
    make_me_zero = zeros(1,3*k.emitter_count + nchoosek(k.emitter_count,2));

    temp = k.uas_ + lambdas_'.*k.pointing_vectors__-emitters_predicted__;
    make_me_zero(1:3*k.emitter_count) = temp(:);
    next_index = 3*k.emitter_count+1;
    for i = 1:k.emitter_count
        for j = i:k.emitter_count
            make_me_zero(next_index) = norm(emitters_predicted__(i,:) - emitters_predicted__(j,:)) - k.distances__(i,j);
            next_index = next_index + 1;
        end
    end
end

function res = simulate_once(k)
    arguments (Input)
        k (1,1) knowns
    end
    % SOLVE
    options = optimset('Display','off','Algorithm','levenberg-marquardt');
    res = fsolve(@(x) fsolve_me(x, k), zeros(1, 16), options);
    
    lambda_1 = res(k.emitter_count*3+1);
    res = reshape(res(1:k.emitter_count*3), [], 3);

    % CORRECT FOR BACKWARDS-NESS
    if (lambda_1 < 0)
        % It went backwards, flip it
        res = -res + 2*k.uas_;
    end
end

function plot_no_error(fignum, k, emitters__)
    arguments
        fignum (1,1) double
        k (1,1) knowns
        emitters__ (:,3) double
    end
    % PLOT IN 3D
    figure(fignum)
    scatter3(k.uas_(1), k.uas_(2), k.uas_(3), 'filled', 'r')
    hold on
    grid minor
    axis equal
    for i = 1:4
        plot3([k.uas_(1), k.uas_(1) + k.pointing_vectors__(i,1)], [k.uas_(2), k.uas_(2) + k.pointing_vectors__(i,2)], [k.uas_(3), k.uas_(3) + k.pointing_vectors__(i,3)], 'k')
        scatter3(emitters__(i,1), emitters__(i,2), emitters__(i,3), 'filled', 'k')
    end
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
end
clc
clear
close all
% INPUTS
r = 0.215;
noise_deg = 0.5;

uas_ = [0 0 10];
rgv_ = [2 0 0];

N = 1000;

plot_for_conditions(1, 2, "#0072BD", r, noise_deg, uas_, rgv_, 4, N);
plot_for_conditions(1, 2, "#D95319", r, noise_deg, uas_, rgv_, 5, N);
plot_for_conditions(1, 2, "#EDB120", r, noise_deg, uas_, rgv_, 6, N);
plot_for_conditions(1, 2, "#7E2F8E", r, noise_deg, uas_, rgv_, 7, N);
plot_for_conditions(1, 2, "#77AC30", r, noise_deg, uas_, rgv_, 8, N);
figure(1)
grid minor
legend(Location="best");
xlabel("Mean Pointing Vector Angle Error [deg]")
ylabel("Mean Emitter Position Error [m]")
xlim([0 noise_deg])
ylim([0 inf])
title("Emitter Position Error vs Angle Error For Different Emitter Counts")
figure(2)
grid minor
legend(Location="best");
xlabel("Mean Pointing Vector Angle Error [deg]")
ylabel("Emitter Position Center Error [m]")
xlim([0 noise_deg])
ylim([0 inf])
title("Emitter Center Error vs Angle Error For Different Emitter Counts")

function plot_for_conditions(fignum1, fignum2, color, d, noise_deg, uas_, rgv_, emitter_count, N)
    % PREP
    emitter_angles = linspace(0,2*pi,emitter_count+1)';
    emitter_angles = emitter_angles(1:end-1);
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
    end
    figure(fignum1)
    hold on
    scatter(rad2deg(mean_noise), mean_error, 4, 'filled', Color=color, DisplayName=sprintf("n=%i", emitter_count), MarkerFaceAlpha = 0.3)
    coeffs = polyfit(rad2deg(mean_noise), mean_error, 1);
    a = coeffs(1);
    b = coeffs(2);
    fplot(@(x) a*x+b, Color=color, DisplayName=sprintf("Fit n=%i, (y=%.2fx+%.2f)", emitter_count, a, b), LineStyle="--", LineWidth=1)
    disp("Mean Emitter Position Error: " + mean(mean_error))
    shg;
    figure(fignum2)
    hold on
    scatter(rad2deg(mean_noise), center_error, 4, 'filled', Color=color, DisplayName=sprintf("n=%i", emitter_count), MarkerFaceAlpha = 0.3)
    coeffs = polyfit(rad2deg(mean_noise), center_error, 1);
    a = coeffs(1);
    b = coeffs(2);
    fplot(@(x) a*x+b, Color=color, DisplayName=sprintf("Fit n=%i, (y=%.2fx+%.2f)", emitter_count, a, b), LineStyle="--", LineWidth=1)
    disp("Mean Center Error: " + mean(center_error))
    shg;
end

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
    res = fsolve(@(x) fsolve_me(x, k), [repmat(k.uas_, 1, k.emitter_count) zeros(1, k.emitter_count)], options);
    
    lambda_1 = res(k.emitter_count*3+1);
    res = reshape(res(1:k.emitter_count*3), [], 3);

    % CORRECT FOR BACKWARDS-NESS
    if (lambda_1 < 0)
        % It went backwards, flip it
        res = -res + 2*k.uas_;
    end
end

% function plot_no_error(fignum, k, emitters__, true_emitters__)
%     arguments
%         fignum (1,1) double
%         k (1,1) knowns
%         emitters__ (:,3) double
%         true_emitters__ (:,3) double
%     end
%     % PLOT IN 3D
%     figure(fignum)
%     scatter3(k.uas_(1), k.uas_(2), k.uas_(3), 'filled', 'r')
%     hold on
%     grid minor
%     axis equal
%     for i = 1:k.emitter_count
%         plot3([k.uas_(1), k.uas_(1) + k.pointing_vectors__(i,1)], [k.uas_(2), k.uas_(2) + k.pointing_vectors__(i,2)], [k.uas_(3), k.uas_(3) + k.pointing_vectors__(i,3)], 'k')
%         scatter3(emitters__(i,1), emitters__(i,2), emitters__(i,3), 'filled', 'k')
%         scatter3(true_emitters__(i,1), true_emitters__(i,2), true_emitters__(i,3), 'filled', 'b')
%     end
%     xlabel("X")
%     ylabel("Y")
%     zlabel("Z")
% end
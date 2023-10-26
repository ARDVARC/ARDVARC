function [Az, El] = ardvarc_samv(steering_vector, iq_samples)
%% Nomenclature
%         SAMV = Sparse Asymptotic Minimum Variance
%         ML = Maximum Likelihood
%% Objective
%
%% Inputs
%         Steering matrix = Each column is a steering vector : a(theta)
%         iq_samples = Complex numbers of wave form reading
%% Outputs
%         Az = Azmith of target
%         El = Elevatino of target
%% Additional Variables
%         sigma = Noise estimate
%         gamma = 
%         error = 
%% Initialize V
    x = iq_samples;
    a = steering_vector;
%% Base Equations
    % Size
    N = max(length(x_vec));
    % Identity matrix
    I = eye(n);
    % First power value
    P(1) = 1 / (A' * inv(diff(R)) * A); 
    % Iteravtive Adaptive Approach (IAA) covariance matrix
    sym x;
    R = @x A * diag(P(x)) * A' + sigma^2 * I; 
%% Power Value Iteration
    while abs(P(x+1) - P(x)) < err
        % SAMV0
        P(x+1) = (A'*R*inv(P(x))*R*R*inv(P(x))*A) / (A'*R*inv(P(x))*A)^2;
        % Update sigma
        sigma = sqrt((R*inv(inv(P(x)))*R).' / (R*inv(inv(P(x)))).');
    end
%% Find Greatest Power
    P_max = max(P,[],'all');
    % Returns [Row,Col]
    [Az,El] = find(P_max == P);
end

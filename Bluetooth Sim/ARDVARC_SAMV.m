function [Az, El] = ARDVARC_SAMV(A)
%% Nomenclature
%         SAMV = Sparse Asymptotic Minimum Variance
%         ML = Maximum Likelihood
%% Objective
%
%% Inputs
%         A = Steering matrix (each column is a steering vector : a(theta))
%% Outputs
%         Az = Azmith of target
%         El = Elevatino of target
%% Additional Variables
%         sigma = Noise estimate
%         gamma = 
%         error = 
%% Base Equations
    % Identity matrix
    I = eye(n);
    % First power value
    P_0(1) = 1 / (A' * inv(diff(R)) * A); 
%     P_1(1) = 1 / (A' * inv(diff(R)) * A); 
%     P_2(1) = 1 / (A' * inv(diff(R)) * A); 
    % Iteravtive Adaptive Approach (IAA) covariance matrix
    sym x;
    R = @x A * diag(P(x)) * A' + sigma^2 * I; 
%% Power Value Iteration
    while abs(P_0(x+1) - P_0(x)) < err
        % SAMV0
        P_0(x+1) = (A'*R*inv(P_0(x))*R*R*inv(P_0(x))*A) / (A'*R*inv(P_0(x))*A)^2;
%         % SAMV1
%         P_1(x+1) = (A'*R*inv(P_1(x))*R*R*inv(P_1(x))*A) / (A'*R*inv(P_1(x))*A);
%         % SAMV2
%         P_2(x+1) = A'*R*inv(P_2(x)) * (R' + gamma*I) * R*inv(P_2(x))*A*p^2; 
        % Update sigma
        sigma = sqrt((R*inv(inv(P_0(x)))*R).' / (R*inv(inv(P_0(x)))).');
%         abs(P_1(x+1) - P_0(x)) < err
%         abs(P_2(x+1) - P_0(x)) < err
    end
%% Find Greatest Power
    P_max = max(P,[],'all');
    % Returns [Row,Col]
    [Az,El] = find(P_max == P);
end
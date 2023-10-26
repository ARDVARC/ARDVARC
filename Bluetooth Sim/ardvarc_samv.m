function [Az, El] = ardvarc_samv(steering_vector, iq_samples)
%% Nomenclature
%         SAMV = Sparse Asymptotic Minimum Variance
%         ML = Maximum Likelihood
%% Objective
%         Use the samv method to converge on a power covariance matrix,
%         from which azmith and elevation can be extracted
%% Inputs
%         Steering matrix = Each column is a steering vector : a(theta)
%         iq_samples = Complex numbers of wave form reading
%% Outputs
%         Az = Azmith of target
%         El = Elevatino of target
%% Initialize Variables
    x = iq_samples;
    a = steering_vector;
    threshold = 1e-6;
    maxIter = 30;
%% Base Equations
    [M, thetaNum] = size(a);
    t_samples = size(x, 2);
    R_N = (x*x')/t_samples;
    % First power value
    p_vec_Old = 1 / (a' * inv(diff(R_N)) * a);
    sigma =  mean(abs(x(:)).^2);
    % Iteration
    for iterIdx = 1:maxIter
        R =  a*spdiags(p_vec_Old, 0, thetaNum, thetaNum)*a' + sigma*eye(M);% 
        Rinv=inv(R);
        Rinv_RN_Rinv_A = Rinv * R_N * Rinv * a;
        p_vec = p_vec_Old.^2 .* sum(conj(a).*Rinv_RN_Rinv_A, 1).';
        sigma = real(trace(Rinv*Rinv*R_N))/real(trace(Rinv*Rinv));
        % Check for convergents
        if norm(p_vec_Old-p_vec)/norm(p_vec_Old)<threshold
            break;
        end
        p_vec_Old = p_vec;
    end
    p_vec = real(p_vec);
%% Find Greatest Power
    P_max = max(p_vec,[],'all');
    % Returns [Row,Col]
    [Az,El] = find(P_max == P);
end

function [mu, sigma] = EKF(mu_prev, u_prev, sigma_prev, z, R, Q, g, Gjac, h, Hjac)
%   INPUTS
%       mu_prev                the previous pose of the robot
%       u_prev                 the control of the robot
%       sigma_prev             the initial sigma 2x2, which is the variance]
%       z                      the sensor measurements
%       R                      the dynamic noise
%       Q                      the sensor measurement noise
%       g                      the function to predict robot pose based on control
%       Gjac                   the jacobian of the g function
%       h                      the function to predict sensor measurement based on predicted pose from g
%       Hjac                   the jacobian of the h function
% 
%   OUTPUTS
%       mu                     the predicted pose of robot
%       sigma                  the predicted variance of the pose of robot
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Nusantara, Jonathan

count = 0;

% Prediction
mu_bar = g(mu_prev, u_prev(1), u_prev(2));
Gt = Gjac(mu_prev, u_prev);

sigma_bar = Gt * sigma_prev * transpose(Gt) + R;

% Update
expected = h(mu_bar);
Ht = Hjac(mu_bar);

notNaN = ~isnan(z); % Get array of boolean of not NaN
if notNaN == 0 % If all NaNs, we should not update the mu and sigma
    mu = mu_prev;
    sigma = sigma_prev;
    
else % If there is non-NaN
    Ht_adjusted = [];
    z_adjusted = [];
    expected_adjusted = [];
    for i = 1:length(z)
        % If sensor measurement not NaN and
        % Expected measurement makes sense, no error caused by predicted pose
        if notNaN(i) == 1
            % This additional condition need to be used for depth measurement
            if (z(i) >= 0.3 && abs((z(i)-expected(i))/expected(i)) < 0.2) || z(i) < 0.3
                count = count + 1;
                Ht_adjusted = [Ht_adjusted; Ht(i,:)];
                z_adjusted = [z_adjusted; z(i)];
                expected_adjusted = [expected_adjusted; expected(i)];
            end
            % Uncomment below for GPS measurement
%             count = count + 1;
%             Ht_adjusted = [Ht_adjusted; Ht(i,:)];
%             z_adjusted = [z_adjusted; z(i)];
%             expected_adjusted = [expected_adjusted; expected(i)];
        end
    end
    
    %  If vector is empty after adjsutment, we should not update the mu and sigma
    if isempty(expected_adjusted) == 1
        mu = mu_prev;
        sigma = sigma_prev;
    else
        Q = Q(1:count,1:count); % Adjust Q based on non-NaN values
    
        kt = sigma_bar * transpose(Ht_adjusted) * inv(Ht_adjusted * sigma_bar * transpose(Ht_adjusted) + Q);

        mu = mu_bar + kt * (z_adjusted - expected_adjusted);
        sigma = (eye(length(mu_prev)) - kt * Ht_adjusted) * sigma_bar;
    end

end
end
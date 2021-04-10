function [newParticles, weights] = PF(oldParticles, u, depthMeasurement, R, Q, g, h)
% testEKF: Performs one step of the extended Kalman Filter, outputs the belief given previous belief
%
%   INPUTS
%       oldParticles    set of initial particles m x 3, only in x y theta
%       u               previous command [d; phi]
%       depth           depth measurement vector, size k
%       R               process noise [x_noise y_noise theta_noise] 3 x 1
%       Q               sensor noise k x 1
%       g               function to calculate odometry, input pose
%       h               function to calculate expected measurement, input pose

%
%   OUTPUTS
%       newParticles    the new particle location
%       weights         the weights of each particle
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Nusantara, Jonathan

m = size(oldParticles, 1); % Number of particles
k = length(depthMeasurement);

% Prediction
x_bar = [];
weights = [];
for i = 1:m % For each particle
    mu_bar = g(oldParticles(i,:), u(1), u(2)); % Calculate odometry, 3 x 1
    x_bar = [x_bar; normrnd(mu_bar(1), sqrt(R(1,1))) normrnd(mu_bar(2), sqrt(R(2,2))) normrnd(mu_bar(3), sqrt(R(3,3)))];
        
    expectedMeasurement = h(x_bar(i,:)); % Vector of expected measurement
    currentWeight = 1;
    for j = 1:k % Iterate through number of sensor measurements
        if ~isnan(depthMeasurement(j)) % If the measurement is not nan
            currentWeight = currentWeight * normpdf(expectedMeasurement(j), depthMeasurement(j), sqrt(Q(j,j))); % Calculate pdf
        end
    end
    weights = [weights currentWeight]; % Add corresponding weights for each prediction
end

% Update
newParticles = [];
if any(weights) == 0
    newParticles = oldParticles;
else
    for i = 1:m
        index = randsample(1:m, 1, true, weights); % Sample for an index based on the weights
        newParticles = [newParticles; x_bar(index,:)];
    end
end

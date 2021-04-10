function[mu_t, sigma_t] = KFStationary(map, sensor_measurement, mu_0, sigma_0)
% gridLocalizationStationary: function to find PDF that represent location
% of robot based on a given map and sensor measurements
% 
% 
%   INPUTS
%       map                     the coordinates of all walls, 4 columns
%       sensor_measurement      sensor measurements from robot, 4 col, NESW
%       mu_0                    the initial pose mu [x; y]
%       sigma_0                 the initial sigma 2x2, which is the variance
% 
%   OUTPUTS
%       mu_t                    the mean for the distribution of location [x; y] 2x1
%       sigma_t                 the variance of the distribution of location 2x2
%
% 
%   Cornell University
%   ECE 5772: Autonomous Mobile Robots
%   Homework #3
%   Nusantara, Jonathan

% Total time (number of iteration)
time_length = size(sensor_measurement,1);

% Sensor measurement size
sensor_measurement_size = size(sensor_measurement);

% Define the matrix
A = [1 0; 0 1]; % Matrix for dynamics, 2x2, identity matrix
% Matrix B is not needed as u_t = 0
% Matrix R for Process noise not needed as it's stationary, 2x2
C_init = [0 -1; -1 0; 0 1; 1 0]; % Measurement: N -> y=-1, E -> x=-1, S -> y=1, W -> x=1, 4x2
% Q = [0.1 0 0 0; 0 0.3 0 0; 0 0 0.1 0; 0 0 0 0.3]; % Measurement noise, 4x4
% Q will be set at every iteration
sensor_noise = [0.1 0.3 0.1 0.3]; % North East South West

% Initialize mu_t and sigma_t
mu_t = mu_0; % 2x1
sigma_t = sigma_0; % Initial uncertainty, should be very uncertain 2x2

% Initialize angles of direction North East South West
direction = [pi/2 0 -pi/2 pi];

% Loop throughout length of t
for t = 1:time_length
    % Prediction: not needed because robot is stationary
    %mu_t_bar = A * mu_t; % No need to add B * u_t = 0
    %sigma_t_bar = A * sigma_t * transpose(A); % R is not calculated as stationary
    
    % Remove NaN measurement and readjust matrices
    % for C, Q, expected, and real measurements
    notNaN = ~isnan(sensor_measurement(t,:)); % Get array of boolean of not NaN
    if notNaN == 0
        continue
    end
    
    C = [];
    Q = [];
    exp_measurement = []; % Vector for expected measurement calculation of non-NaN  directions
    z_t = []; % Vector for sensor measurement of non-NaN  directions
    for i = 1:sensor_measurement_size(2) % For the number of directions
        if notNaN(i) == 1 % If a measurement is non-NaN for that direction
            % Add that C entry from the initial C
            C = [C; C_init(i,:)];
            % Add the sensor_noise entry with non-NaN to Q_temp, which will then be diag
            Q = [Q sensor_noise(i)];
            % Calculate and input expected measurement only for non-NaN directions
            % Calculation is based on pose and map
            exp_measurement = [exp_measurement; depthPredict([mu_t(1) mu_t(2) direction(i)],map,[0 0],0)];
            % Extract only non-NaN measurements
            z_t = [z_t; sensor_measurement(t,i)];
        end
    end
    Q = diag(Q); % Diagonalize the noise that we have
    
    % Update
    % Calculate kalman gain
    k_t = sigma_t * transpose(C) * inv(C * sigma_t * transpose(C) + Q);
    
    mu_t = mu_t + k_t * (z_t - exp_measurement);
    sigma_t = (eye(2) - k_t * C) * sigma_t;
    
end


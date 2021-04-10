function[p_kt] = gridLocalizationStationary(map, sensor_measurement, grid_size)
% gridLocalizationStationary: function to find PDF that represent location
% of robot based on a given map and sensor measurements
% 
% 
%   INPUTStype
%       map                     the coordinates of all walls, 4 columns
%       sensor_measurement      sensor measurements from robot, 4 col, NESW
%       grid_size               size of the grid for the map, (x,y)
% 
%   OUTPUTS
%       p_kt                    the pdf representing the location of the robot
%
% 
%   Cornell University
%   ECE 5772: Autonomous Mobile Robots
%   Homework #3
%   Nusantara, Jonathan

% Total number of cells
cell_total = grid_size(1) * grid_size(2);

% Total time (number of iteration)
time_length = length(sensor_measurement);

% Get coordinate of map max and min values, and its size
map_min_x = min(min(map(:,1)), min(map(:,3)));
map_min_y = min(min(map(:,2)), min(map(:,4)));
map_max_x = max(max(map(:,1)), max(map(:,3)));
map_max_y = max(max(map(:,2)), max(map(:,4)));
map_width = map_max_x - map_min_x;
map_height = map_max_y - map_min_y;

% Initialize the PDF with uniform distribution
p_kt = ones(1, cell_total) / cell_total;

% Define cell
% Robot location in grid is in the middle of cell
% Given location at cell x, the x coordinate is 
% ((x * cell_width) - (cell_width / 2))
cell_width = map_width / grid_size(1);
cell_height = map_height / grid_size(2);

% Loop throughout length of t
for t = 1:time_length
    % Initialize p_kt_bar that is zeroed on every time loop
    %p_kt_bar = zeros(1, cell_total);
    
    % Prediction
    % Calculate the sum of probability across all cells
    % p_kt_bar = sigma_i (p(x_kt|x_it-1,u_t) * p_it-1 )
    % But since u_t = 0 (stationary), 
    % p(x_kt|x_it-1,u_t) > 0 only if k = i
    % So p(x_kt|x_kt-1,u_t) given u_t=0 is 1
    % So p_kt+1_bar = p_kt
    
    
    % Update
    % For each cell k
    for k = 1:cell_total
        % p_kt = normalization * p(z_t|x_kt) * p_kt_bar
        % There are 4 p(z_t|x_kt) for a total of 4 directions: NESW
        p_zt_xkt = zeros(1,4);
        
        % Find the row and column of the cell based on k
        cell_col = mod(k,grid_size(1)); % x axis
        cell_row = ceil(k / grid_size(2)); % y axis
        if cell_col == 0 % If result is 0, means the grid_size(1)th column
            cell_col = grid_size(1);
        end
        
        % Find the current robot location coordinate (middle of cell)
        pose_x = (cell_col * cell_width) - (cell_width / 2);
        pose_y = (cell_row * cell_height) - (cell_height / 2);
                
        % Find the prob of real measurement given pose and map
        direction = [pi/2 0 -pi/2 pi]; % From facing E at 0, list NESW
        for dir = 1:4
            % Calculate expected measurement using depthPredict
            exp_measurement = depthPredict([pose_x pose_y direction(dir)],map,[0 0],0);
                        
            % Get the measured sensor value as mu
            mu = sensor_measurement(t,dir);
            
            % Decide on sigma value of the noise in sensor measurement
            if dir == 1 || dir == 3 % North or South
                sigma = sqrt(.1);
            else % East or West
                sigma = sqrt(.3);
            end
            
            % Calculate the measurement probability at that direction
            % PROBABILITY CAN BE BIGGER THAN 0?
            % WHAT IF NAN?
            if isnan(mu) % Nothing detected by the sensor
                p_zt_xkt(dir) = 1 ; % Set it = 1 to not ruin calculation
%                 if exp_measurement > 3 % If expected is more than 3m
%                     p_zt_xkt(dir) = 1 ; % Set probability to 1
%                 else % If expected is less than 3m
%                     p_zt_xkt(dir) = normpdf(exp_measurement,3,sigma); % mu is set to 3
%                 end
            else
                p_zt_xkt(dir) = normpdf(exp_measurement,mu,sigma);
            end
        end
        % Multiplied by measurement probability from all 4 sides
        % Multiplied with p_kt, as it is equal to p_kt_bar when stationary
        p_kt(k) = p_zt_xkt(1) * p_zt_xkt(2) * p_zt_xkt(3) * p_zt_xkt(4) * p_kt(k);
    end
    % Normalize to have total sum equal to 1
    p_kt = p_kt / sum(p_kt);
end


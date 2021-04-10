function [logOddsGrid] = logOddsDepth(robotPose, depth, L0, numCells, boundary)
% return the grid containing the log odds for each cell
%
%   INPUTS
%       robotPose    robot's pose [x y theta] through time (N-by-3)
%       depth        depth sensor measurements through time (N-by-9)
%       L0           prior initialization value of logOdds for each cell
%       numCells     number of cells in X and Y direction in the grid [X Y]
%       boundary     vector representing boundary of the grids [Xstart Xend Ystart Yend]

% 
%   OUTPUTS
%       logOddsGrid     final log odds for Y by X grid cells (Y-by-X)
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Jonathan Nusantara
%   Homework #5



% Define measurement time length
time_length = size(robotPose,1);

% Define cell
cell_width = (boundary(2) - boundary(1)) / numCells(1); % (Xend-Xstart) / # of cells in X
cell_height =(boundary(4) - boundary(3)) / numCells(2); % (Yend-Ystart) / # of cells in Y

% Find x and y midpoint for each cell to calculate closest cell of coordinates
cellMid_x = linspace(boundary(1),boundary(2),numCells(1)+1);
cellMid_x = cellMid_x(1:end-1); % Remove last entry
cellMid_x = cellMid_x + cell_width/2; % Shift to midpoint
cellMid_y = linspace(boundary(3),boundary(4),numCells(2)+1);
cellMid_y = cellMid_y(1:end-1); % Remove last entry
cellMid_y = cellMid_y + cell_height/2; % Shift to midpoint

% Initialize log odds grid cells based on size dimension input
logOddsGrid = zeros(numCells(2), numCells(1));

% Calculate ;eft to right angles offset of depth sensor
angles = linspace(27*pi/180,-27*pi/180, 9); 

% Variable to find points on depth measurement to find free space
% Use the smaller of width and height to find points in every cell passed
increment_diag = min(cell_width, cell_height);

for t = 1:time_length
    % Update log odds based on current robot pose
    % Calculate current grid cell location based on robot pose
    [~,index_x] = min(abs(cellMid_x - robotPose(t,1)));
    [~,index_y] = min(abs(cellMid_y - robotPose(t,2)));
    robotPoseCell = [index_x index_y];
    
    % Update log odds grid for the cell of current robot pose
    logOddsGrid(robotPoseCell(2),robotPoseCell(1)) = logOddsGrid(robotPoseCell(2),robotPoseCell(1)) - 0.368 - L0;
    
    % Update log odds based on detected objects from depth measurement
    % For each direction of depth measurement
    for i = 1:size(depth,2)
        if depth(t,i) ~= 0 && depth(t,i) < 10 % Depth of 0 or bigger than 10 is not accurate
            % Find global coordinate of detected object from depth measurement (1 by 2)
            currentDepthGlobal = robot2global(robotPose(t,1:3),[0.13+depth(t,i) depth(t,i)*tan(angles(i))]);
            
            % Find cell number in grid for robot pose and bump sensor locations
            [~,index_x] = min(abs(cellMid_x - currentDepthGlobal(1)));
            [~,index_y] = min(abs(cellMid_y - currentDepthGlobal(2)));
            currentDepthCell = [index_x index_y];
            
            % Update log odds at calculated cell
            logOddsGrid(currentDepthCell(2),currentDepthCell(1)) = logOddsGrid(currentDepthCell(2),currentDepthCell(1)) + 5 - L0;
        end
        
        if depth(t,i) ~= 0 % Ignore depth of 0, but can still learn from depth > 10 despite inaccuracy
            % Update log odds based on free space learned from depth measurement
            % Calculate proportional xy increments from diagonal increment, based on min of cell dimension
            increment_x = increment_diag*cos(angles(i)); 
            increment_y = increment_diag*sin(angles(i));
            current_diag = increment_diag;
            current_x = 0.13; % Start from offset of depth sensor in robot frame
            current_y = 0;
            range = depth(t,i) / cos(angles(i));
            
            % Loop to calculate log odds for every meaningful points on depth sensor trajectory
            while current_diag < range
                % Update current x and y by adding increment
                current_x = current_x + increment_x;
                current_y = current_y + increment_y;
                
                % Find global coordinate of free space from depth measurement (1 by 2)
                currentDepthGlobal = robot2global(robotPose(t,1:3),[current_x current_y]);
                
                % Calculate current grid cell location based on robot pose
                [~,index_x] = min(abs(cellMid_x - currentDepthGlobal(1)));
                [~,index_y] = min(abs(cellMid_y - currentDepthGlobal(2)));
                currentDepthCell = [index_x index_y];
                
                % Update log odds grid for the cell of current depth
                logOddsGrid(currentDepthCell(2),currentDepthCell(1)) = logOddsGrid(currentDepthCell(2),currentDepthCell(1)) - 0.176 - L0;
                
                % Update current diagonal(range) by adding increment
                current_diag = current_diag + increment_diag;
            end  
        end
    end
end
end
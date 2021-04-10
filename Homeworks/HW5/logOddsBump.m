function [logOddsGrid] = logOddsBump(robotPose, bump, L0, numCells, boundary)
% return the grid containing the log odds for each cell
%
%   INPUTS
%       robotPose    robot's pose [x y theta] through time (N-by-3)
%       bump         bump sensor measurements through time [right left front] (N-by-3)
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


logOddsGrid = zeros(numCells(2), numCells(1));

% Calculation of offsets of bump sensor in robot frame
% Side bump sensor is assumed to make contact at 45deg from x axis robot frame
bumpOffsetx = 0.16 * sind(45);
bumpOffsety = 0.16 * cosd(45);

for t = 1:time_length
    % Find global coordinate of right, left, and front bump sensor (1 by 2)
    % Robot radius = 0.16m
    rightBumpGlobal = robot2global(robotPose(t,1:3),[bumpOffsetx -bumpOffsety]);
    leftBumpGlobal = robot2global(robotPose(t,1:3),[bumpOffsetx bumpOffsety]);
    frontBumpGlobal = robot2global(robotPose(t,1:3),[0.16 0]);
    
    % Find cell number in grid for robot pose and bump sensor locations
    [~,index_x] = min(abs(cellMid_x - robotPose(t,1)));
    [~,index_y] = min(abs(cellMid_y - robotPose(t,2)));
    robotPoseCell = [index_x index_y];
    
    [~,index_x] = min(abs(cellMid_x - rightBumpGlobal(1)));
    [~,index_y] = min(abs(cellMid_y - rightBumpGlobal(2)));
    rightBumpCell = [index_x index_y];
    
    [~,index_x] = min(abs(cellMid_x - leftBumpGlobal(1)));
    [~,index_y] = min(abs(cellMid_y - leftBumpGlobal(2)));
    leftBumpCell = [index_x index_y];
    
    [~,index_x] = min(abs(cellMid_x - frontBumpGlobal(1)));
    [~,index_y] = min(abs(cellMid_y - frontBumpGlobal(2)));
    frontBumpCell = [index_x index_y];
    
    % Update log odds of each cell
    % logodds_t(mi) = logodds_t-1(mi) + sensor model - L0
    if bump(t,1) == 1 % Right bump sensor
        logOddsGrid(rightBumpCell(2),rightBumpCell(1)) = logOddsGrid(rightBumpCell(2),rightBumpCell(1)) + 5 - L0;
    else
        logOddsGrid(rightBumpCell(2),rightBumpCell(1)) = logOddsGrid(rightBumpCell(2),rightBumpCell(1)) - 0.368 - L0;
    end
    
    if bump(t,2) == 1 % Left bump sensor
        logOddsGrid(leftBumpCell(2),leftBumpCell(1)) = logOddsGrid(leftBumpCell(2),leftBumpCell(1)) + 5 - L0;
    else
        logOddsGrid(leftBumpCell(2),leftBumpCell(1)) = logOddsGrid(leftBumpCell(2),leftBumpCell(1)) - 0.368 - L0;
    end
    
    if bump(t,3) == 1 % Front bump sensor
        logOddsGrid(frontBumpCell(2),frontBumpCell(1)) = logOddsGrid(frontBumpCell(2),frontBumpCell(1)) + 5 - L0;
    else
        logOddsGrid(frontBumpCell(2),frontBumpCell(1)) = logOddsGrid(frontBumpCell(2),frontBumpCell(1)) - 0.368 - L0;
    end
    
    logOddsGrid(robotPoseCell(2),robotPoseCell(1)) = logOddsGrid(robotPoseCell(2),robotPoseCell(1)) - 0.368 - L0;
    
end
end
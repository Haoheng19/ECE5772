function [lFinalBump,lFinalDepth]=TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
% TESTOCCUPANCYGRID 
% Test function for MAE 4180/5180 CS 3758, Homework 5. 
% Returns and plots the final occupancy grids using the bump and depth sensors.
% Will create two (2) figures containing the occupancy grids.
%
%       TestOccupancyGrid(dataStore,l_0,NumCellsX,NumCellsY,boundaryX,boundaryY)
%
%       INPUTS:
%           dataStore   struct from running SimulatorGUI
%           l_0         initial log odds (scalar value)
%           NumCellsX   number of cells in the x direction (integer)
%           NumCellsY   number of cells in the y direction (integer)
%           boundaryX   boundary of the environment in the x direction.
%                       1 x 2 array [min_x max_x]
%           boundaryY   boundary of the environment in the Y direction.
%                       1 x 2 array [min_y max_y]
%       OUTPUTS:
%           lFinalBump  Final occupancy grid using bump sensor
%           lFinalDepth Final occupancy grid using depth sensor
%       Figures Created:
%           Figure 1    Occupancy grid using bump sensor
%           Figure 2    Occupancy grid from depth information

% Cornell University
% Autonomous Mobile Robots
% Homework #5
% Nusantara, Jonathan

% Calculate log odds
lFinalBump = logOddsBump(dataStore.truthPose(:,2:4), [dataStore.bump(:,2) dataStore.bump(:,3) dataStore.bump(:,7)], l_0, [NumCellsX NumCellsY], [boundaryX boundaryY])
lFinalDepth = logOddsDepth(dataStore.truthPose(:,2:4), dataStore.rsdepth(:,3:11), l_0, [NumCellsX NumCellsY], [boundaryX boundaryY])

% Define cell
cell_width = (boundaryX(2) - boundaryX(1)) / NumCellsX; % (Xend-Xstart) / # of cells in X
cell_height =(boundaryY(2) - boundaryY(1)) / NumCellsY; % (Yend-Ystart) / # of cells in Y

% Define cell midpoints
tempx = linspace(boundaryX(1)+cell_width/2,boundaryX(2)-cell_width/2,NumCellsX);
tempy = linspace(boundaryY(1)+cell_height/2,boundaryY(2)-cell_height/2,NumCellsY);

% Create mesh grid
[meshX,meshY] = meshgrid(tempx,tempy);
meshX = reshape(meshX,1,[]);
meshY = reshape(meshY,1,[]);

% Plot occupancy grid
figure;
plotOccupancyGrid(meshX, meshY, lFinalBump);
title([int2str(NumCellsY) 'x' int2str(NumCellsX) ' Occupancy grid based on bump sensor'])

figure;
plotOccupancyGrid(meshX, meshY, lFinalDepth);
title([int2str(NumCellsY) 'x' int2str(NumCellsX) ' Occupancy grid based on depth sensor'])

end
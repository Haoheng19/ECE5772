function[lidarXY] = lidar_range2xy(lidarR, robotRad, angRange)
% LIDAR_RANGE2XY: convert lidar measurements (range & bearing) into x/y
% coordinates (in robot local frame)
% 
%   LIDARXY = LIDAR_RANGE2XY(LIDARR,ROBOTRAD,ANGRANGE) returns the
%   x/y coordinates (in robot local frame) of lidar measurements.
% 
%   INPUTS
%       lidarR      1-by-N vector of scan ranges (meters)
%       robotRad    robot radius (meters)
%       angRange    total angular field of view of lidar (radians) (1-by-2)
% 
%   OUTPUTS
%       lidarXY     2-by-N matrix of x/y scan locations
% 
%   NOTE: Assume lidar is located at front of robot and pointing forward 
%         (along robot's x-axis).
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Nusantara, Jonathan 

% Lidar reading length
l = length(lidarR);

% Angle measurement
angle = linspace(angRange(1), angRange(2), l);

% Store lidarXY matrix
lidarXY = [robotRad + lidarR .* cos(angle); lidarR .* sin(angle)];

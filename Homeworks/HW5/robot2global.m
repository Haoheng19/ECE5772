function[xyG] = robot2global(pose,xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
% 
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
% 
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Nusantara, Jonathan 

% Define local variables from pose input
px = pose(1);
py = pose(2);
theta = pose(3);

% Define T_IB
T_IB = [cos(theta) -sin(theta) px; sin(theta) cos(theta) py; 0 0 1];

% Calculate xyG
calc = T_IB * [transpose([xyR]); 1];
xyG = transpose(calc(1:2));
end


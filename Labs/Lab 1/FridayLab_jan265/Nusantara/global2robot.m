function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
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

% Define T_BI
T_BI = inv([cos(theta) -sin(theta) px; sin(theta) cos(theta) py; 0 0 1]);

% Calculate xyR
calc = T_BI * [transpose([xyG]); 1];
xyR = transpose(calc(1:2));

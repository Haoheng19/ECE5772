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
%   Agarwal, Aarushi
    
    t = pose(1,3);
    Tgr = [cos(t), -1*sin(t), pose(1,1); sin(t), cos(t), pose(1,2); 0, 0, 1];
    w = [xyR(1,1); xyR(1,2); 1];
    
    calc = Tgr*w;
    
    xyG = [calc(1,1), calc(2,1)];

end
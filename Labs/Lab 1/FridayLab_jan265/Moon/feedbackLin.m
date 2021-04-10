function [cmdV,cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% feedbackLin: find the forward and angular velocities that accomplish
% going to [cmdVx, cmdVy] using feadback linearization.
% 
%   cmdV,cmdW = feedbackLin(POSE,XYG) returns the forward and angular 
%   velocities that accomplish going to [cmdVx, cmdVy] using feadback 
%   linearization.
% 
%   INPUTS
%       cmdVx    x-direction in inertial frame an omnidirectional robot
%                would take
%       cmdVy    y-direction in inertial frame an omnidirectional robot
%                would take
%       theta    robot's current measure of rotation in pose
%       epsilon  small distance to allow rotation
% 
%   OUTPUTS
%       cmdV     forward velcity to achieve desired movement
%       cmdW     angular velcity to achieve desired movement
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Moon, Jonathan 
R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
A = [1, 0; 0, 1/epsilon];
T = A*R;
inputs = [cmdVx, cmdVy].';
outputs = T*inputs;
cmdV = outputs(1);
cmdW = outputs(2);
end

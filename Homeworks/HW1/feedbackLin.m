function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
%   INPUTS
%       cmdVx       x intertial direction velocity command
%       cmdVy       y intertial direction velocity command
%       theta       angle
%       epsilon     epsilon
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Nusantara, Jonathan

% Calculate cmdV and cmdW using inputs
temp = [1 0; 0 1/epsilon] * [cos(theta) sin(theta); -sin(theta) cos(theta)] * [cmdVx; cmdVy];
cmdV = temp(1);
cmdW = temp(2);


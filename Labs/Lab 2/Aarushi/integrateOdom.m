function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2

    current = initPose;
    finalPose = [];
    for i = 1:length(d)
        dc = d(1,i);
        phic = phi(1,i);
        t = current(3,1);
        
        if phic ~= 0        
            dx = (dc/phic) * (sin(t+phic)-sin(t));
            dy = -1*(dc/phic) * (cos(t+phic)-cos(t));
        else            
            dx = dc * cos(t);
            dy = dc * sin(t);
        end
        
        current = [current(1,1) + dx; current(2,1) + dy; t+phic];
        finalPose = horzcat(finalPose, current);
                
    end
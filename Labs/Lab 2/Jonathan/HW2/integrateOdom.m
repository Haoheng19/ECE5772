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
%   Jonathan Nusantara
%   Homework #2

index = 1;
l = length(d);
current_x = initPose(1);
current_y = initPose(2);
current_theta = initPose(3);

while index <= l
    if phi(index) == 0 % Move in a straight line
        next_x = current_x + (d(index) * cos(current_theta));
        next_y = current_y + (d(index) * sin(current_theta));
        next_theta = current_theta;
  
    else % If there is an angle / not moving straight
        next_x = current_x + (d(index)/phi(index)) * (sin(current_theta + phi(index))-sin(current_theta));
        next_y = current_y - (d(index)/phi(index)) * (cos(current_theta + phi(index))-cos(current_theta));
        next_theta = current_theta + phi(index);
    end
    
    % Set the finalPose value
    finalPose(1,index) = next_x;
    finalPose(2,index) = next_y;
    finalPose(3,index) = next_theta;
    
    % Update current pose
    current_x = next_x;
    current_y = next_y;
    current_theta = next_theta;
    
    % Update index
    index = index + 1;
end


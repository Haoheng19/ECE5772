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
    N = length(d);
    finalPose = zeros(3,N+1);
    finalPose(:,1) = initPose;
    for i = 1:N
        angle = phi(i);
        if angle == 0
            y = 0;
            x = d(i);
        else
            r = abs(d(i)/angle);
            x = abs(sin(angle)*r)*sign(d(i));
            y = (r-sqrt(r^2-x^2))*sign(phi(i));
        end
        theta = finalPose(3,i);
        d_i = [cos(theta) -sin(theta); sin(theta) cos(theta)] * [x;y];
        finalPose(:,i+1) = [finalPose(1:2,i)+d_i;
                            finalPose(3,i)+angle];
    end
    finalPose = finalPose(:,2:end);
end
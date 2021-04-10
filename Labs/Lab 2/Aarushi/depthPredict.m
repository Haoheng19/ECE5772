function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
    
    sensorPose = robot2global(robotPose', sensorOrigin);
    
    maxRange = 3.5; 
    depth = [];

    for i=1:length(angles)
        dist = maxRange;
        ang = robotPose(3) + angles(i,1);
        sensSeg = [sensorPose(1), sensorPose(2), sensorPose(1)+(maxRange*cos(ang)), sensorPose(2)+(maxRange*sin(ang))];
        mSize = size(map);
        lines = mSize(1);
        for j=1:lines
            [inter,~,~,d] = intersectPoint(sensSeg(1),sensSeg(2),sensSeg(3),sensSeg(4),map(j,1),map(j,2),map(j,3),map(j,4));
            if(inter && d < dist)
                dist = d;
            end
        end
        de = dist * cos(angles(i)) * maxRange;
        depth(i,1) = de;
    end
            
end
        
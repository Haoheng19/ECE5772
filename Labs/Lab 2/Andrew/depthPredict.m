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
%   cannot possibly be bigger
    maxLength = 2*(max(max(map))-min(min(map)));
    depth = ones(size(angles)) * maxLength;
    for i = 1:length(angles)
        theta = robotPose(3);
        matIR = [cos(theta) -sin(theta) robotPose(1);
                 sin(theta) cos(theta)  robotPose(2);
                 0 0 1];
        theta = angles(i);
        matRS = [cos(theta) -sin(theta) sensorOrigin(1);
                 sin(theta) cos(theta)  sensorOrigin(2);
                 0 0 1];
        for j = 1:size(map,1)
            point1 = matRS\(matIR\[map(j,1:2).';1]);
            point2 = matRS\(matIR\[map(j,3:4).';1]);
            [isect,x,~,~]= intersectPoint(0,0,depth(i),0,point1(1),point1(2),point2(1),point2(2));
            if isect && depth(i)>x*cos(angles(i))
                depth(i) = x*cos(angles(i));
            end
        end
    end    
end
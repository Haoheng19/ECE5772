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
%   Jonathan Nusantara
%   Homework 2

LINE_LENGTH = 10000; % Constant for line length of sensor trajectory
length_angles = 9;
length_map = size(map);
% Calculate the location of sensor in global coordinate
sensor_xy = robot2global(transpose(robotPose), sensorOrigin);
depth = zeros(length_angles,1); % Multiplied with very big amount

% For each angle
for i = 1:length_angles
    % Calculate the other coordinate point of the sensor direction
    sensor_xy_other = [(sensor_xy(1) + LINE_LENGTH * cos(robotPose(3)+angles(i))) (sensor_xy(2) + LINE_LENGTH * sin(robotPose(3)+angles(i)))];
    for j = 1:length_map(1) % For each map lines
        % Calculate the intersection
        [isect_bool,x_temp,y_temp,temp_range]= intersectPoint(sensor_xy(1),sensor_xy(2),sensor_xy_other(1),sensor_xy_other(2),map(j,1),map(j,2),map(j,3),map(j,4));
        if isect_bool == 1 && ((sqrt((x_temp-sensor_xy(1))^2 + (y_temp-sensor_xy(2))^2) * cos(angles(i))) < depth(i) || depth(i) == 0)% If intersects and closer depth than previous
            depth(i) = sqrt((x_temp-sensor_xy(1))^2 + (y_temp-sensor_xy(2))^2) * cos(angles(i)); % Store depth value based on range

        end
    end
end
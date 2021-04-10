function H = HjacDepth(x, map, sensor_pos, K)
% HjacDepth: output the jacobian of the depth measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       map          of environment, n x [x1 y1 x2 y2] walls
%       sensor_pos   sensor position in the body frame [1x2]
%       K            number of measurements (rays) the sensor gets between 27 to -27 
%
%   OUTPUTS
%       H            Kx3 jacobian matrix
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Lin, Andrew
    delta = 0.00001;
    dp = @(pose) depthPredict(pose,map,sensor_pos,linspace(deg2rad(27),deg2rad(-27),K)');
    depth_ref =dp(x);
    depth_x = dp(x+[delta;0;0]);
    depth_y = dp(x+[0;delta;0]);
    depth_the = dp(x+[0;0;delta]);
    H = [(depth_x-depth_ref)/delta (depth_y-depth_ref)/delta (depth_the-depth_ref)/delta];
end
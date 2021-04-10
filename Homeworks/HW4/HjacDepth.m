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
%   Nusantara, Jonathan

angles = linspace(27*pi/180,-27*pi/180, K);
depth_initial = depthPredict(x, map, sensor_pos, angles);
eps = 1e-5;

H = zeros(K,3);

for i = 1:3
    x_perturbation = x;
    x_perturbation(i) = x_perturbation(i) + eps;
    H(:,i) = (depthPredict(x_perturbation, map, sensor_pos, angles) - depth_initial) / eps;
end
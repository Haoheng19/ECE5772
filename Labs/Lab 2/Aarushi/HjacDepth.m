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
%   Last, First Name
    angs = linspace((27 * pi / 180), (-27 * pi / 180), K)';
    fin = 10^-6;
    H = zeros(length(angs));
    H = H(:,1:3);
    depth = depthPredict(x, map, sensor_pos, angs);
    H1 = depthPredict([x(1)+fin, x(2), x(3)]', map, sensor_pos, angs) - depth;
    H2 = depthPredict([x(1), x(2)+fin, x(3)]', map, sensor_pos, angs) - depth;
    H3 = depthPredict([x(1), x(2), x(3)+fin]', map, sensor_pos, angs) - depth;
    H = [H1 H2 H3] / fin;
end
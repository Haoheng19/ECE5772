function Loc = hGPS(x)
% hGPS: predict the GPS measurements for a robot pose.
%
%   Loc = hGPS(pose) returns
%   the expected GPS measurements
%
%   INPUTS
%       x            3-by-1 vector of pose
%
%   OUTPUTS
%       Loc          n-by-m vector of the output of the GPS
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Nusantara, Jonathan

% The prediction in the GPS will be equal to robot pose
Loc = [1 0 0; 0 1 0; 0 0 1] * x;

end
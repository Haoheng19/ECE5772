function H = HjacGPS(x)
% HjacGPS: output the jacobian of the GPS measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%
%   OUTPUTS
%       H            jacobian matrix of GPS measurement
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Nusantara, Jonathan
    
% [dh(1)/dx dh(1)/dy dh(1)/dtetha
%  dh(2)/dx dh(2)/dy dh(2)/dtetha
%  dh(3)/dx dh(3)/dy dh(3)/dtetha]

H = [1 0 0; 0 1 0; 0 0 1];

end
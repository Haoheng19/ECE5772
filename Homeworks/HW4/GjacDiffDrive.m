function G = GjacDiffDrive(x, u)
% GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       u            2-by-1 vector [d, phi]'
%
%   OUTPUTS
%       G            Jacobian matrix partial(g)/partial(x)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Nusantara, Jonathan

if u(2) == 0 % If phi == 0
    G = [1 0 (u(1))*(-sin(x(3))); 0 1 (u(1))*(cos(x(3))); 0 0 1];
else % If phi != 0
    G = [1 0 (u(1)/u(2))*(cos(x(3)+u(2))-cos(x(3))); 0 1 (u(1)/u(2))*(sin(x(3)+u(2))-sin(x(3))); 0 0 1];
end
    
end
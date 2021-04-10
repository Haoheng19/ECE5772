function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Nusantara, Jonathan 

% Calculate left and right wheel velocity
velLeft = fwdVel - wheel2Center * angVel;
velRight = fwdVel + wheel2Center * angVel;

% Scaling wheel velocity based on max velocity
if abs(velLeft) > maxV || abs(velRight) > maxV
    tempMax = max(abs(velLeft), abs(velRight));
    velLeft = velLeft / tempMax * maxV;
    velRight = velRight / tempMax * maxV;
end

% Calculate cmdV and cmdW
cmdV = 0.5 * (velRight + velLeft);
cmdW = (velRight - velLeft) / (2 * wheel2Center);
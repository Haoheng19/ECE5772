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
%   Lin, Andrew
    Vl = fwdVel-angVel*wheel2Center;
    Vr = fwdVel+angVel*wheel2Center;
    scaleFactor = max(max(abs(Vl),abs(Vr))/maxV,1);
    Vl = Vl/scaleFactor;
    Vr = Vr/scaleFactor;
    cmdV = (Vr+Vl)/2;
    cmdW = (Vr-Vl)/wheel2Center/2;
end

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
%   Agarwal, Aarushi

    L = 2*wheel2Center;
    Vr = fwdVel + (L * angVel/2);
    Vl = fwdVel - (L * angVel/2);
    
    if abs(Vr) <= maxV  && abs(Vl) <= maxV
        cmdV = fwdVel;
        cmdW = angVel;
    else
        x = max([abs(Vr),abs(Vl)]);

        Vr = Vr/x* maxV;
        Vl = Vl/x* maxV;
        
        cmdW = 1/L * (Vr - Vl);
        cmdV = .5* (Vr+Vl);
        
    end
            
end
   

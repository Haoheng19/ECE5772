function[poseData] = OpenLoopControl(Robot)
% OpenLoopControl: program to control the iRobot Create and collect pose
% data. The data is collected before and after every motion.
%  
%   poseData = OpenLoopControl(Robot) runs 
% 
%   INPUT
%       Robot       Port configurations and robot name (get from running CreatePiInit)
% 
%   OUTPUT
%       poseData    struct containing logged pose data
%
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots


% declare poseData as a global variable so it can be accessed from the
% workspace even if the program is stopped
global poseData;

% initialize poseData to current pose 
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[px, py, pt];

% Moving in a straight line
% First using travelDist, 3 different velocities
travelDist(Robot.CreatePort, 0.1, 0.5)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
travelDist(Robot.CreatePort, 0.2, 0.5)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
travelDist(Robot.CreatePort, 0.4, 0.5)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
% Second using linear velocity, at 2 different velocities
SetFwdVelAngVelCreate(Robot.CreatePort, 0.1,0);
pause(5)
SetFwdVelAngVelCreate(Robot.CreatePort, 0,0)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
SetFwdVelAngVelCreate(Robot.CreatePort, 0.5,0);
pause(1)
SetFwdVelAngVelCreate(Robot.CreatePort, 0,0)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];

% Turning at different angular velocities
% First using turn angle, 4 different velocities
turnAngle(Robot.CreatePort, 0.05, 90)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
turnAngle(Robot.CreatePort, 0.1, 90)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
turnAngle(Robot.CreatePort, 0.2, 90)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
turnAngle(Robot.CreatePort, 0.3, 90)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];

% Second, by setting angular velocity, at 2 different velocities
SetFwdVelAngVelCreate(Robot.CreatePort, 0,0.2);
pause(8)
SetFwdVelAngVelCreate(Robot.CreatePort, 0,0)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];
SetFwdVelAngVelCreate(Robot.CreatePort, 0,0.5)
pause(4)
SetFwdVelAngVelCreate(Robot.CreatePort, 0,0)
[px, py, pt] = OpenloopLocalization(Robot);
poseData=[poseData; px, py, pt];

% Custom:
cmdFwdVel = 0.5;
cmdAngVel = 0;
MAX_VEL = 0.5;
WHEEL2CENTER=0.13;
[cmdFwdVel, cmdAngVel] = limitCmds(cmdFwdVel, cmdAngVel, MAX_VEL, WHEEL2CENTER);
SetFwdVelAngVelCreate(Robot.CreatePort, cmdFwdVel, cmdAngVel);
pause(2)
SetFwdVelAngVelCreate(Robot.CreatePort, 0,0)

end

function [px, py, pt] = OpenloopLocalization(Robot)
    % number of attempts to get localization
    N = 10;
    % returns NaNs if timed out
    px = NaN; py = NaN; pt = NaN;
    while N>0
        try
            [px, py, pt] = OverheadLocalizationCreate(Robot);
            break
        catch
            N = N - 1;
        end
    end
end
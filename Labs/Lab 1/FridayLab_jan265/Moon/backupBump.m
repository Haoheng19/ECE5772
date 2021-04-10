function[dataStore] = backupBump(Robot,maxTime)
% backupBump: A program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot move and backup when hitting a
% wall.
% 
%   dataStore = backupBump(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
MAXVEL = 0.5;
WHEEL2CENTER = 0.13;

SetFwdVelAngVelCreate(CreatePort, 0,0);
tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    % CONTROL FUNCTION (send robot commands)
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        bumpRight = dataStore.bump(end, 2);
        bumpLeft = dataStore.bump(end, 3);
        bumpFront = dataStore.bump(end, end);
        % if one of the bumpers is activated, backup .25 meters and rotate 30
        % degrees.
        if bumpRight || bumpLeft || bumpFront 
            [cmdV,~]= limitCmds(MAXVEL, 0, MAXVEL, WHEEL2CENTER);
            travelDist(CreatePort, cmdV, -0.25);
            [cmdV,cmdW]= limitCmds(0, MAXVEL, MAXVEL, WHEEL2CENTER)
            turnAngle(CreatePort,cmdW, -30);
        % otherwise, move forward with a slight curve.
        else  
            cmdV = 0.6;
            cmdW = -0.25;
            [cmdV,cmdW]= limitCmds(cmdV, cmdW, MAXVEL, WHEEL2CENTER);
        end
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    pause(0.1);
end



% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

function[dataStore] = visitWaypoints(Robot,maxTime)
% VISITWAYPOINTS: program to use with iRobot Create (or simulator) that
% navigates to different waypoints.
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = VISITWAYPOINTS(Robot,maxTime) travels to the waypoint and 
%               wrap around the different waypoints after finishing.
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

% CONSTANT DEFINITIONS
% *********************
% WAYPOINTS is a n*2 dimensional array of waypoints.
% CLOSE_ENOUGH is the distance that is suitable to 
% MAXVEL is the maximum velocity of each wheel.
% WHEEL2CENTER is the distance from the wheels to the center of the robot
%   (in meters)
% EPSILON is the minimum distance for rotation to happen.
WAYPOINTS = [-3, 0; 0, -3; 3, 0; 0, 3];
EPSILON = 0.2;
CLOSE_ENOUGH = 0.1;
maxV = 0.49;
wheel2Center = 0.13;
gotopt = 0;

SetFwdVelAngVelCreate(CreatePort, 0,0);
tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    currPos = [dataStore.truthPose(end, 2), dataStore.truthPose(end, 3)];
    % if at a close enough distance, move to next waypoint
    if (dist(WAYPOINTS(gotopt+1, :), currPos) < CLOSE_ENOUGH)
        gotopt = mod(gotopt + 1, length(WAYPOINTS));
    else    
        currWaypoint = WAYPOINTS(gotopt+1, :)
        theta = dataStore.truthPose(end, 4);
        % In order to prevent overshooting, scale down the velocity of the 
        % robot as it approaches the waypoint.
        Vx=currWaypoint(1)-currPos(1);
        Vy=currWaypoint(2)-currPos(2);
        [cmdV, cmdW] = feedbackLin(Vx, Vy, theta, EPSILON);
        [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
        % if overhead localization loses the robot for too long, stop it
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(CreatePort, 0,0);
        else
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
        end
        pause(0.01);
    end
end
    
% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
end

function dist = dist(p1, p2)
    dist = sqrt(sum((p1-p2).^2));
end

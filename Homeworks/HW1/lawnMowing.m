function[dataStore] = lawnMowing(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, move in a lawn-mowing path, and saves a datalog.
% 
%   dataStore = lawnMowing(Robot,maxTime) runs 
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
%   Homework #1
%   Nusantara, Jonathan

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 100;
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

% Initialization of local variable
%counter = 1
dir = 0 % Variable for cw or ccw direction
distance = 0 % Variable for long or short distance
turn_count = 0 % Variable for to count number of turns done, will reset at 2
state = "long";


SetFwdVelAngVelCreate(CreatePort, 0,0);
tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
        
    elseif state == "rotate"
        if dir == 0 % Clockwise
            [cmdV,cmdW] = limitCmds(0, 0.15, 0.49, 0.13);
            % Intended to turn 90deg, but it would went over,
            % so 88.5 works best
            turnAngle(CreatePort, cmdW, -88.5);
        else % Anti-clockwise
            [cmdV,cmdW] = limitCmds(0, 0.15, 0.49, 0.13);
            turnAngle(CreatePort, cmdW, 88.5);
        end
        turn_count = turn_count + 1;
        if turn_count == 2
            dir = ~dir;
            turn_count = 0;
            state = "long";
        else
            state = "short";
        end
        
    elseif state == "short" % Move forward for a short distance
        [cmdV,cmdW] = limitCmds(0.5, 0, 0.49, 0.13);
        travelDist(CreatePort, cmdV, 0.25); % Travel for 0.25m
        state = "rotate";

    else % state == "long"
        [cmdV,cmdW] = limitCmds(0.5, 0, 0.49, 0.13);
        travelDist(CreatePort, cmdV, 1); % Travel for 1m
        state = "rotate";
    end
    
    pause(0.1);
    %counter = counter + 1; % Update loop counter
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

figure
%dataStore.truthPose(:,2:3)
plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3))
%hold on
%plot(dataStore.truthPose(:,3),'r-')
%hold off
legend('Robot trajectory')
title('Robot trajectory during lawn-mowing program')
xlabel('x coordinate intertial frame') 
ylabel('y coordinate intertial frame')
savefig('plot_trajectory_lawnmowing_function885.fig')

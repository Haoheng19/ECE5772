% function[dataStore] = backupBump(Robot,maxTime)
% % TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% % Reads data from sensors, makes the robot backup and rotate when bumping, and saves a datalog.
% % 
% %   dataStore = backupBump(Robot,maxTime) runs 
% % 
% %   INPUTStype
% %       Robot       Port configurations and robot name (get from running CreatePiInit)
% %       maxTime     max time to run program (in seconds)
% % 
% %   OUTPUTS
% %       dataStore   struct containing logged data
% 
% % 
% %   NOTE: Assume differential-drive robot whose wheels turn at a constant 
% %         rate between sensor readings.
% % 
% %   Cornell University
% %   MAE 5180: Autonomous Mobile Robots
% %   Homework #1
% %   Nusantara, Jonathan
% 
% % Set unspecified inputs
% if nargin < 1
%     disp('ERROR: TCP/IP port object not provided.');
%     return;
% elseif nargin < 2
%     maxTime = 45;
% end
% 
% try 
%     % When running with the real robot, we need to define the appropriate 
%     % ports. This will fail when NOT connected to a physical robot 
%     CreatePort=Robot.CreatePort;
% catch
%     % If not real robot, then we are using the simulator object
%     CreatePort = Robot;
% end
% 
% % declare dataStore as a global variable so it can be accessed from the
% % workspace even if the program is stopped
% global dataStore;
% 
% % initialize datalog struct (customize according to needs)
% dataStore = struct('truthPose', [],...
%                    'odometry', [], ...
%                    'rsdepth', [], ...
%                    'bump', [], ...
%                    'beacon', []);
% 
% 
% % Variable used to keep track of whether the overhead localization "lost"
% % the robot (number of consecutive times the robot was not identified).
% % If the robot doesn't know where it is we want to stop it for
% % safety reasons.
% noRobotCount = 0;
% 
% % Initialization of local variable
% counter = 1;
% bump = 0;
% state = "";
% 
% 
% SetFwdVelAngVelCreate(CreatePort, 0,0);
% tic
% while toc < maxTime
%     
%     % READ & STORE SENSOR DATA
%     [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
%     
%     % CONTROL FUNCTION (send robot commands)
%     
%     % Set velocity
%     %cmdV = 0.3;
%     %cmdW = 0;
% 
%     % if overhead localization loses the robot for too long, stop it
%     if noRobotCount >= 3
%         SetFwdVelAngVelCreate(CreatePort, 0,0);
%         
%     elseif state == "backward"
%         [cmdV,cmdW] = limitCmds(0.3, 0, 0.1, 0.13);
%         travelDist(CreatePort, cmdV, -0.25); % Function to move robot certain dist
%         state = "rotate";
%         
%     elseif state == "rotate"
%         [cmdV,cmdW] = limitCmds(0, 0.15, 0.1, 0.13);
%         turnAngle(CreatePort, cmdW, -30); % Function to rotate robot certain angle
%         state = "move";
%         
%     else % state == "move"
%         [cmdV,cmdW] = limitCmds(0.3, 0, 0.1, 0.13);
%         SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
%         % Check if bump sensor detects anything
%         bump = dataStore.bump(counter, 2) || dataStore.bump(counter, 3) || dataStore.bump(counter, 4) || dataStore.bump(counter, 5) || dataStore.bump(counter, 6) || dataStore.bump(counter, 7);
%         if bump == 1
%             SetFwdVelAngVelCreate(CreatePort, 0, 0 );
%             lastX = dataStore.truthPose(counter, 2);
%             lastY = dataStore.truthPose(counter, 3);
%             lastTheta = dataStore.truthPose(counter, 4);
%             state = "backward";
%         end
%         
%     end
%     
%     pause(0.1);
%     counter = counter + 1; % Update loop counter
% end
% 
% % set forward and angular velocity to zero (stop robot) before exiting the function
% SetFwdVelAngVelCreate(CreatePort, 0,0 );
% figure
% %dataStore.truthPose(:,2:3)
% plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3))
% %hold on
% %plot(dataStore.truthPose(:,3),'r-')
% %hold off
% legend('Robot trajectory')
% title('Robot trajectory during backup program')
% xlabel('x coordinate intertial frame') 
% ylabel('y coordinate intertial frame')
% savefig('plot_trajectory_backup_function.fig')

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
    maxTime = 45;
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
MAXVEL = 0.1;
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
            [cmdV,cmdW]= limitCmds(0, MAXVEL, MAXVEL, WHEEL2CENTER);
            turnAngle(CreatePort,cmdW, -30);
        % otherwise, move forward with a slight curve.
        else  
            cmdV = 0.6;
            cmdW = 0;
            [cmdV,cmdW]= limitCmds(cmdV, cmdW, MAXVEL, WHEEL2CENTER);
        end
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    pause(0.1);
end



% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

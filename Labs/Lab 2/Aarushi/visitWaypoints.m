function[dataStore] = visitWaypoints(Robot, maxTime)

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

SetFwdVelAngVelCreate(CreatePort, 0,0);
tic

gotopt = 1;
e = .2;
closeEnough = .1;
waypoints = [-1,0;1,0]; %[-3, 0;0,-3;3,0;0,3];

while gotopt < length(waypoints) + 1 && toc < maxTime
    
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    pose = dataStore.truthPose;
    
    waypoint = waypoints(gotopt,:);
    
    % set vars
    while (pdist( [waypoint(1,1), waypoint(1,2); pose(end,2),pose(end,3) ], 'euclidean') > closeEnough)
        
        % READ & STORE SENSOR DATA
        [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
        pose = dataStore.truthPose;
        
        theta = pose(end, 4);
        
        Vx = waypoint(1,1) - pose(end,2);
        Vy = waypoint(1,2) - pose(end,3);
    
        [cmdV, cmdW] = feedbackLin(Vx,Vy,theta,e);
        [cmwV, cmdW] = limitCmds(cmdV, cmdW, .5, .13);
    
    
        SetFwdVelAngVelCreate(CreatePort, cmwV, cmdW);
        
        %pause(0.1);
        
    end
    
    gotopt = gotopt + 1;

    

end

SetFwdVelAngVelCreate(CreatePort, 0,0 );


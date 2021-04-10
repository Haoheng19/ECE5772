function[dataStore] = backupBump(Robot, maxTime)

if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end

global dataStore;
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);

noRobotCount = 0;
SetFwdVelAngVelCreate(CreatePort, 0,0);
tic
while toc < maxTime
    
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    % check if bump
    [a,b,~,~,~,f] = BumpsWheelDropsSensorsRoomba(CreatePort);
    sensed = a || b || f;
    
    if sensed == 1
        travelDist(CreatePort, .3, -.25);
        pause(1);
        turnAngle(CreatePort, .2, -30);
        pause(1);
    else
        SetFwdVelAngVelCreate(CreatePort, 1, 0);
    end
    pause(0.1);
end
SetFwdVelAngVelCreate(CreatePort, 0,0 );
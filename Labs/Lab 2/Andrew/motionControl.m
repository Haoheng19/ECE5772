function[dataStore] = motionControl(Robot,maxTime)
% MOTIONCONTROL: Move forward at constant velocity until it bumps into something.
% If a bump sensor is triggered, command the robot to back up 0.25m
% and turn counter-clockwise 30 degrees, before continuing to drive forward again.
% 
%   dataStore = motionControl(Robot,maxTime) runs 
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
    maxTime = 60;
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
dataStore = struct('truthPose', [], ...
                   'GPS', [], ...
                   'odometry', [], ...
                   'deadReck', [], ...
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'particles', [], ...
                   'particlesWeights', [], ...
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
maxSpeed = 0.2;
wheel2Center = 0.13;
while toc < maxTime
    toc
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    
    if dataStore.bump(end,end) + dataStore.bump(end,2) + dataStore.bump(end,3) ~= 0 
        travelDist(CreatePort, maxSpeed, -0.25);
        turnAngle(CreatePort, 0.2, 30)
    end
    
    % Set angular velocity
    cmdV = 1;
    cmdW = 0.5;
    
    % limit wheel speed
    [cmdV,cmdW] = limitCmds(cmdV,cmdW,maxSpeed,wheel2Center);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );

R_var = 0.01;
Q_var = 0.00001;

R = eye(3)*R_var;
correct_init = 1;
% dead reckoning
if correct_init == 1
    dataStore.deadReck = integrateOdom(...
                          dataStore.truthPose(1,2:4)',...
                          dataStore.odometry(:,2)',...
                          dataStore.odometry(:,3)')';
else
    dataStore.deadReck = integrateOdom(...
                      [-2;-1.5;pi/2],...
                      dataStore.odometry(:,2)',...
                      dataStore.odometry(:,3)')';
end
    
measurement_option = 2; % 1 for GPS, 2 for Depth
filter_option = 1; % 1 for EKF, 2 for PF

% GPS
if measurement_option == 1
    Q = eye(3)*Q_var;
    dataStore.GPS = normrnd(dataStore.truthPose(:,2:end), sqrt(Q_var));
else
    load('lab2WallMap.mat')
    map = lab2WallMap;
    n_depth = length(dataStore.rsdepth(1,:))-2;
    sensor_pos = [0.08 0];
    Q = eye(n_depth)*Q_var;
end

% EKF
if filter_option == 1
    if measurement_option == 1
        Hjac = @(x) HjacGPS(x);
        h = @(x) hGPS(x);
    else
        Hjac = @(x) HjacDepth(x, map, sensor_pos, n_depth);
        h = @(x) depthPredict(x, map, sensor_pos, linspace(deg2rad(27),deg2rad(-27),n_depth)');
    end
    g = @(x, u) integrateOdom(x,u(1),u(2));
    Gjac = @(x, u) GjacDiffDrive(x, u);
    % estimate the state with GPS data using EKF
    if correct_init == 1
        dataStore.ekfMu = dataStore.truthPose(1,2:end);
        dataStore.ekfSigma = [0.05 0 0 0 0.05 0 0 0 0.1];
    else
        dataStore.ekfMu = [-2 -1.5 pi/2];
        dataStore.ekfSigma = [4 0 0 0 4 0 0 0 0.01];
    end
    for i = 2:length(dataStore.odometry)
        if measurement_option == 1
            z = dataStore.GPS(i, :)';
        else
            z = dataStore.rsdepth(i, 3:end)';
        end
        [mu, sigma] = EKF(dataStore.ekfMu(i-1, :)',...
            dataStore.odometry(i,2:end),...
            reshape(dataStore.ekfSigma(i-1, :),[3 3])...
            , g, Gjac, R, z, h, Hjac, Q);

        dataStore.ekfMu = [dataStore.ekfMu ; ...
                          mu'];
        dataStore.ekfSigma = [dataStore.ekfSigma ; ...
                          reshape(sigma, [1 9])];
    end
else %PF
    g = @(x, u) integrateOdom(x, u(1), u(2))';
    n_particles = 50;
    dataStore.particles = reshape(...
    [   ones([n_particles,1])*dataStore.truthPose(1,2) ...
        ones([n_particles,1])*dataStore.truthPose(1,3) ...
        ones([n_particles,1])*dataStore.truthPose(1,4) ...
        ones([n_particles,1])] ...
        ,[1 n_particles*4]);
    w = @(x, z, map) particleWeight(x, z, map, 0.5);
    for i = 2:length(dataStore.odometry)
        i
        z = dataStore.rsdepth(i, 3:end)';
        R_par = R*10;
        particles = PF(...
        reshape(dataStore.particles(i-1, :),...
        [n_particles 4]),g,dataStore.odometry(i,2:end),...
        R_par,w,z,map);
        dataStore.particles = [dataStore.particles ; ...
            reshape(particles, [1 n_particles*4])];
    end
end

    

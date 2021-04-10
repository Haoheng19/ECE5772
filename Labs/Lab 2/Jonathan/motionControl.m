function[dataStore] = motionControl(Robot,maxTime)
% MOTIONCONTROL: control the motion of the robot while calculating EKF or PF
% Reads data from sensors, makes the robot backup and rotate when bumping, and saves a datalog.
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
%   Homework #4
%   Nusantara, Jonathan


%
%
%
% Before contiuing with the program, select a filter mode
% Select a filter mode from the following
% Extended Kalman Filter with GPS Measurement = Mode 1
% Extended Kalman Filter with Depth Measurement = Mode 2
% Particle Filter with Depth measurement = Mode 3
% NOTE: Input the integer only, and not the word 'Mode"
filterMode = 3;
%
%
%


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 150;
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
                   'beacon', [], ...
                   'deadReck', [], ...
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'GPS', [], ...
                   'particles', []);
               
               
% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

% CONSTANTS
MAX_VEL = 0.2; % Max velocity of the robot
WHEEL2CENTER = 0.13; % Distance of wheel to center of robot
DIRECTIONS = 9; % Number of directions in depth measurements
ANGLES = linspace(27*pi/180,-27*pi/180, DIRECTIONS); % Calculate angles
SENSOR_POS = [0.13 0]; % Position of sensor on robot frame

% Initialization of local variable
state = ""; % 3 states: moving, rotate, or bckward
time_started = 0;
map = load('lab2WallMap.mat');
map = map.lab2WallMap;

% Initialization of filter-related variables
R = 0.1 * eye(3); % Process noise
% Measurement noise
if filterMode == 1
    Q = 0.001 * eye(3); % For x y theta of the GPS
    
else
    % In postlab 1, std is 0.005 so var is 0.000025 
    Q = 0.000025 * eye(9); % For the 9 angles NEED TO REVERT BACK TO 0.001
end

SetFwdVelAngVelCreate(CreatePort, 0,0);

tic % Start program timer

% Initialize deadReck, ekfMu, ekfSigma
[noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
dataStore.deadReck = [toc dataStore.truthPose(end, 2:4)];
%dataStore.deadReck = [toc -2 -1.5 pi/2];
if filterMode == 3 % Mode 3 for PF
    % Particle size = 20
    initialParticle = zeros(50,3);
    initialParticle(:,1) = ones(1,50) * dataStore.truthPose(end, 2); % x
    initialParticle(:,2) = ones(1,50) * dataStore.truthPose(end, 3); % y
    initialParticle(:,3) = ones(1,50) * dataStore.truthPose(end, 4); % theta
    weights = ones(1,50); % Uniform weights
%     initialParticle(:,1) = unifrnd(-4.5,-3.5,1,20);
%     initialParticle(:,2) = unifrnd(1.5,2.5,1,20);
%     initialParticle(:,3) = unifrnd(-0.2,0.2,1,20);


    % Particle size = 500
%     initialParticle = zeros(500,3);
%     initialParticle(:,1) = unifrnd(-5,0,1,500);
%     initialParticle(:,2) = unifrnd(-5,5,1,500);
%     initialParticle(:,3) = unifrnd(-0.2,0.2,1,500);
%     weights = ones(1,500); % Uniform weights
    
    % Initialize particles pose and weight
    dataStore.particles.x = [toc transpose(initialParticle(:,1))];
    dataStore.particles.y = [toc transpose(initialParticle(:,2))];
    dataStore.particles.theta = [toc transpose(initialParticle(:,3))];
    dataStore.particles.weight = [toc weights];

else % Mode 1 or 2 for EKF
    dataStore.ekfMu = [toc dataStore.truthPose(end, 2:4)];
%     dataStore.ekfMu = [toc -2 -1.5 pi/2];
    dataStore.ekfSigma = [toc 0.05 0 0 0 0.05 0 0 0 0.1];
%     dataStore.ekfSigma = [toc 4 0 0 0 4 0 0 0 0.02];
end


while toc < maxTime
    toc
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    
    % Perform filter based on chosen filter
    % Common code through different modes
    g = @(initPose, d, phi) integrateOdom(initPose, d, phi); % Pointer to integrateOdom
    Gjac = @(x, u) GjacDiffDrive(x, u); % Pointer to GjacDiffDrive
    dataStore.deadReck = [dataStore.deadReck; toc transpose(integrateOdom(transpose(dataStore.deadReck(end, 2:4)), dataStore.odometry(end,2), dataStore.odometry(end,3)))];
    
    % Mode 1: Extended Kalman Filter on GPS
    if filterMode == 1
        dataStore.GPS = [dataStore.GPS; toc normrnd(dataStore.truthPose(end, 2), sqrt(Q(1,1))) normrnd(dataStore.truthPose(end, 3), sqrt(Q(2,2))) normrnd(dataStore.truthPose(end, 4), sqrt(Q(3,3)))];
        h_gps = @(mu_bar) hGPS(mu_bar); % Pointer to hGPS
        Hjac_gps = @(mu_bar) HjacGPS(mu_bar); % Pointer to HjacGPS
        % Call filter function
        
        [mu_gps, sigma_gps] = EKF(transpose(dataStore.ekfMu(end,2:end)), transpose(dataStore.odometry(end,2:end)), reshape(dataStore.ekfSigma(end,2:end),3,3), transpose(dataStore.GPS(end,2:end)), R, Q, g, Gjac, h_gps, Hjac_gps);
        % Store prediction
        dataStore.ekfMu = [dataStore.ekfMu; toc reshape(mu_gps,1,3)];
        dataStore.ekfSigma = [dataStore.ekfSigma; toc reshape(sigma_gps, 1, 9)];
        
    % Mode 2: Extended Kalman Filter on Depth measurement
    elseif filterMode == 2
        h_depth = @(mu_bar) depthPredict(mu_bar,map,SENSOR_POS,ANGLES); % Pointer to depthPredict
        Hjac_depth = @(x) HjacDepth(x, map, SENSOR_POS, DIRECTIONS); % Pointer to Hjac
        z = transpose(dataStore.rsdepth(end,3:end));
        for i = 1:length(z)
            if z(i) < 0.175 % Below 0.175m, sensor cannot measure depth
                z(i) = NaN;
            end
        end
        % Call filter function
        [mu_depth, sigma_depth] = EKF(transpose(dataStore.ekfMu(end,2:end)), transpose(dataStore.odometry(end,2:end)), reshape(dataStore.ekfSigma(end,2:end),3,3), z, R, Q, g, Gjac, h_depth, Hjac_depth);
        % Store prediction
        dataStore.ekfMu = [dataStore.ekfMu; toc reshape(mu_depth,1,3)];
        dataStore.ekfSigma = [dataStore.ekfSigma; toc reshape(sigma_depth, 1, 9)];
        
    % Mode 3: Particle Filter on Depth measurement    
    elseif filterMode == 3
        h_depth = @(mu_bar) depthPredict(mu_bar,map,SENSOR_POS,ANGLES); % Pointer to depthPredict
        z = transpose(dataStore.rsdepth(end,3:end));
        for i = 1:length(z)
            if z(i) < 0.175 % Below 0.175m, sensor cannot measure depth
                z(i) = NaN;
            end
        end
        % Call filter function
        [newParticles, weights] = PF(transpose([dataStore.particles.x(end,2:end); dataStore.particles.y(end,2:end); dataStore.particles.theta(end,2:end)]), transpose(dataStore.odometry(end,2:end)), z, R, Q, g, h_depth);
        % Store prediction
        dataStore.particles.x = [dataStore.particles.x; toc transpose(newParticles(:,1))];
        dataStore.particles.y = [dataStore.particles.y; toc transpose(newParticles(:,2))];
        dataStore.particles.theta = [dataStore.particles.theta; toc transpose(newParticles(:,3))];
        dataStore.particles.weight = [dataStore.particles.weight; toc weights];
    end 
    
        
    % CONTROL FUNCTION (send robot commands)
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
        
    elseif state == "backward"
        if time_started == 0 % If time has not been started
            start_time = tic;
            [cmdV,cmdW] = limitCmds(-0.3, 0, MAX_VEL, WHEEL2CENTER);
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
            time_started = 1;
        end
        
        if toc(start_time) > 0.75
            SetFwdVelAngVelCreate(CreatePort, 0, 0 );
            time_started = 0;
            state = "rotate";
        end
        
    elseif state == "rotate"
        if time_started == 0 % If time has not been started
            start_time = tic;
            [cmdV,cmdW] = limitCmds(0, 0.4, MAX_VEL, WHEEL2CENTER);
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
            time_started = 1;
        end
        if toc(start_time) > 1.5
            SetFwdVelAngVelCreate(CreatePort, 0, 0 );
            time_started = 0;
            state = "move";
        end

    else % state == "move"
        [cmdV,cmdW] = limitCmds(0.3, 0, MAX_VEL, WHEEL2CENTER);
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
        % Check if bump sensor detects anything
        bump = dataStore.bump(end, 2) || dataStore.bump(end, 3) || dataStore.bump(end, 4) || dataStore.bump(end, 5) || dataStore.bump(end, 6) || dataStore.bump(end, 7);
        if bump == 1
            SetFwdVelAngVelCreate(CreatePort, 0, 0 );
            state = "backward";
        end
        
    end
    
    pause(0.1);
end

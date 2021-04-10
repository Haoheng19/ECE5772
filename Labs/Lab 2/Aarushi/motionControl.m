function[dataStore] = motionControl(Robot, maxTime)

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
                   'beacon', [], ...
                   'deadReck', [], ...
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'GPS', [], ...
                   'particles', [], ...
                   'tier5', [], ...
                   'big1', [] ...
                   );

noRobotCount = 0;
SetFwdVelAngVelCreate(CreatePort, 0,0);
tic

while isempty(dataStore.truthPose)
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    pause(.1);
end

%while the below are specific to different filters and siutations, 
% only Q needs to be commented/uncommented depending on what is running
mu_0 = dataStore.truthPose(1,2:4); %this could be truthpose or just predefined vector
dataStore.deadReck = mu_0;
dataStore.ekfMu = mu_0;
dataStore.ekfSigma = [.05 0 0; 0 .05 0; 0 0 .1];
R = .01 * eye(3); % for all, this is process noise
% Q = .001 * eye(3); % using GPS data, this is measurement noise
Q = .0001 * eye(9); % using depth data, this is measurement noise
dataStore.GPS = dataStore.truthPose(1,2:4) + (randn(size(Q, 1)) * diag(Q));
num_part = 200;
X = ones(num_part, 1) * dataStore.truthPose(1,2);
Y = ones(num_part, 1) * dataStore.truthPose(1,3);
th = ones(num_part, 1) * dataStore.truthPose(1,4);
dataStore.particles = [X, Y, th];

map = load('lab2WallMap.mat');
map = map.lab2WallMap;
n_rs_rays = 9;
sensor_pos = [.13 0];


dynamics = @(x,u) GjacDiffDrive(x, u);
JacDepth = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays); 
measureGPS = @(x) hGPS(x);
JacGPS = @(x) HjacGPS(x);
angs = linspace((27 * pi / 180), (-27 * pi / 180), n_rs_rays)';
measureDepth = @(x) depthPredict(x, map, sensor_pos, angs);
predict = @(mu, u) integrateOdom(mu, u(1), u(2));
EKF_step = @(mu_prev, u_prev, sigma_prev, z, Hfunc, hfunc)...
        EKF(mu_prev, u_prev, sigma_prev, R, z, Q, predict, dynamics, Hfunc, hfunc);
pf = @(X_t, u, z) PF(X_t, u, z, R, Q, measureDepth, predict);

while toc < maxTime
    
    % check if bump
    [a,b,~,~,~,f] = BumpsWheelDropsSensorsRoomba(CreatePort);
    sensed = a || b || f;
    
    if sensed == 1
        turnAngle(CreatePort, .2, 40);
    else
        [cmwV, cmdW] = limitCmds(6, -2, .1, .13);
        SetFwdVelAngVelCreate(CreatePort, cmwV, cmdW);
    end
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    u = dataStore.odometry(end,2:3);
    dataStore.deadReck = [dataStore.deadReck; predict( (dataStore.deadReck(end,:)'), u )'];
    %EKF SECTION
    %next four lines if GPS
%     [mu, sigma] = EKF_step(dataStore.ekfMu(end,:)', dataStore.odometry(end,2:3)', dataStore.ekfSigma(end-2:end,:), dataStore.GPS(end,:)', JacGPS, measureGPS);
%     dataStore.GPS = [dataStore.GPS; dataStore.truthPose(end,2:4) + (randn(size(Q, 1)) * diag(Q))];
%     dataStore.ekfMu = [dataStore.ekfMu; mu'];
%     dataStore.ekfSigma = [dataStore.ekfSigma; sigma];
    %next line if depth data
    [mu, sigma] = EKF_step(dataStore.ekfMu(end,:)', dataStore.odometry(end,2:3)', dataStore.ekfSigma(end-2:end,:), dataStore.rsdepth(end,3:11)', JacDepth, measureDepth); 
    dataStore.ekfMu = [dataStore.ekfMu; mu'];
    dataStore.ekfSigma = [dataStore.ekfSigma; sigma];
    %END OF EKF
    
    %PARTICLE FILTER SECTION
    %next line if depth data
%     [x_t, tier5, big1] = pf(dataStore.particles(end-num_part+1:end,:), dataStore.odometry(end,2:3)', dataStore.rsdepth(end,3:11)');
%     dataStore.particles = [dataStore.particles; x_t];
%     dataStore.tier5 = [dataStore.tier5; tier5];
%     dataStore.big1 = [dataStore.big1; big1];
    
    
    pause(0.1);
end
SetFwdVelAngVelCreate(CreatePort, 0,0 );
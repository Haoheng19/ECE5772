% Before contiuing with the program, select a filter mode
% Select a filter mode from the following
% Extended Kalman Filter with GPS Measurement = Mode 1
% Extended Kalman Filter with Depth Measurement = Mode 2
% Particle Filter with Depth measurement = Mode 3
% NOTE: Input the integer only, and not the word 'Mode"
filterMode = 1;
%
%
%
               
% CONSTANTS
DIRECTIONS = 9; % Number of directions in depth measurements
ANGLES = linspace(27*pi/180,-27*pi/180, DIRECTIONS); % Calculate angles
SENSOR_POS = [0 0.08]; % Position of sensor on robot frame

% Initialization of filter-related variables
R = 0.01 * eye(3); % Process noise ANDREW USES 0.1, I USUALLY USE 0.01, NEED TO BE SAME
% Measurement noise
if filterMode == 1
    Q = 0.001 * eye(3); % For x y theta of the GPS
    
else
    % In postlab 1, std is 0.005 so var is 0.000025 
    Q = 0.000025 * eye(9); % For the 9 angles NEED TO REVERT BACK TO 0.001 or 0.000025
end

% Initialize deadReck, ekfMu, ekfSigma
dataStore.GPS = []
dataStore.ekfMu = []
dataStore.ekfSigma = []
%dataStore.deadReck = []
dataStore.particles = []

dataStore.deadReck = [dataStore.truthPose(1,1) dataStore.truthPose(1, 2:4)];
if filterMode == 3 % Mode 3 for PF
    % Particle size = 50
    initialParticle = zeros(5000,3);
    initialParticle(:,1) = ones(1,5000) * dataStore.truthPose(1, 2); % x
    initialParticle(:,2) = ones(1,5000) * dataStore.truthPose(1, 3); % y
    initialParticle(:,3) = ones(1,5000) * dataStore.truthPose(1, 4); % theta
    weights = ones(1,5000); % Uniform weights
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
    dataStore.particles.x = [dataStore.truthPose(1,1) transpose(initialParticle(:,1))];
    dataStore.particles.y = [dataStore.truthPose(1,1) transpose(initialParticle(:,2))];
    dataStore.particles.theta = [dataStore.truthPose(1,1) transpose(initialParticle(:,3))];
    dataStore.particles.weight = [dataStore.truthPose(1,1) weights];

else % Mode 1 or 2 for EKF
%     dataStore.ekfMu = [dataStore.truthPose(1,1) dataStore.truthPose(1, 2:4)];
    dataStore.ekfMu = [dataStore.truthPose(1,1) 1 -2.5 0];
%     dataStore.ekfMu = [dataStore.truthPose(1,1) 0 2.5 0];
    dataStore.ekfSigma = [dataStore.truthPose(1,1) 0.05 0 0 0 0.05 0 0 0 0.1];
%     dataStore.ekfSigma = [toc 4 0 0 0 4 0 0 0 0.02];
end

% Calculation to match depth timestamp to odom timestamp
currentOdomIndex = 1;
odomIndexList = [];
trueDepthTimes = dataStore.rsdepth(:,1) - dataStore.rsdepth(:,2); % Recorded time - delay
for i = 1:length(trueDepthTimes) % For the entire length of depth measurement
    while dataStore.odometry(currentOdomIndex,1) < trueDepthTimes(i,1) % Iterate until odom timestamp is bigger than depth timestamp
        currentOdomIndex = currentOdomIndex + 1; % and update the index
    end
    if currentOdomIndex == 1 % If currentOdomIndex-1 == 0, we change it to 1, as ther eis no index 0
        currentOdomIndex = 2;
    end
    odomIndexList = [odomIndexList; currentOdomIndex-1]; % Store the index
end

for i = 2:length(dataStore.odometry)-1
    % Calculate the actual odometry by interpolation
    [~,currentPoseIndex] = min(abs(dataStore.truthPose(:,1) - dataStore.odometry(i,1)))
    
    % Perform filter based on chosen filter
    % Common code through different modes
    g = @(initPose, d, phi) integrateOdom(initPose, d, phi); % Pointer to integrateOdom
    Gjac = @(x, u) GjacDiffDrive(x, u); % Pointer to GjacDiffDrive
    dataStore.deadReck = [dataStore.deadReck; dataStore.odometry(i,1) transpose(integrateOdom(transpose(dataStore.deadReck(i-1, 2:4)), dataStore.odometry(i,2), dataStore.odometry(i,3)))];
    
    % Mode 1: Extended Kalman Filter on GPS
    if filterMode == 1
        dataStore.GPS = [dataStore.GPS; dataStore.odometry(i,1) normrnd(dataStore.truthPose(currentPoseIndex-1, 2), sqrt(Q(1,1))) normrnd(dataStore.truthPose(currentPoseIndex-1, 3), sqrt(Q(2,2))) normrnd(dataStore.truthPose(currentPoseIndex-1, 4), sqrt(Q(3,3)))];
        h_gps = @(mu_bar) hGPS(mu_bar); % Pointer to hGPS
        Hjac_gps = @(mu_bar) HjacGPS(mu_bar); % Pointer to HjacGPS
        % Call filter function
        
        [mu_gps, sigma_gps] = EKF(transpose(dataStore.ekfMu(i-1,2:end)), transpose(dataStore.odometry(i,2:end)), reshape(dataStore.ekfSigma(i-1,2:end),3,3), transpose(dataStore.GPS(end,2:end)), R, Q, g, Gjac, h_gps, Hjac_gps);
        % Store prediction
        dataStore.ekfMu = [dataStore.ekfMu; dataStore.odometry(i,1) reshape(mu_gps,1,3)];
        dataStore.ekfSigma = [dataStore.ekfSigma; dataStore.odometry(i,1) reshape(sigma_gps, 1, 9)];
        
    % Mode 2: Extended Kalman Filter on Depth measurement
    elseif filterMode == 2
        h_depth = @(mu_bar) depthPredict(mu_bar,lab2WallMap,SENSOR_POS,ANGLES); % Pointer to depthPredict
        Hjac_depth = @(x) HjacDepth(x, lab2WallMap, SENSOR_POS, DIRECTIONS); % Pointer to Hjac
        z = transpose(dataStore.rsdepth(i,3:end));
        for j = 1:length(z)
            if z(j) < 0.175 % Below 0.175m, sensor cannot measure depth
                z(j) = NaN;
            end
        end
        % Call filter function
        [mu_depth, sigma_depth] = EKF(transpose(dataStore.ekfMu(end,2:end)), transpose(dataStore.odometry(odomIndexList(i),2:end)), reshape(dataStore.ekfSigma(end,2:end),3,3), z, R, Q, g, Gjac, h_depth, Hjac_depth);
        % Store prediction
        dataStore.ekfMu = [dataStore.ekfMu; dataStore.odometry(i,1) reshape(mu_depth,1,3)];
        dataStore.ekfSigma = [dataStore.ekfSigma; dataStore.odometry(i,1) reshape(sigma_depth, 1, 9)];
        
    % Mode 3: Particle Filter on Depth measurement    
    elseif filterMode == 3
        h_depth = @(mu_bar) depthPredict(mu_bar,lab2WallMap,SENSOR_POS,ANGLES); % Pointer to depthPredict
        z = transpose(dataStore.rsdepth(i,3:end));
        for j = 1:length(z)
            if z(j) < 0.175 % Below 0.175m, sensor cannot measure depth
                z(j) = NaN;
            end
        end
        % Call filter function
        [newParticles, weights] = PF(transpose([dataStore.particles.x(end,2:end); dataStore.particles.y(end,2:end); dataStore.particles.theta(end,2:end)]), transpose(dataStore.odometry(odomIndexList(i),2:end)), z, R, Q, g, h_depth);
        % Store prediction
        dataStore.particles.x = [dataStore.particles.x; dataStore.odometry(i,1) transpose(newParticles(:,1))];
        dataStore.particles.y = [dataStore.particles.y; dataStore.odometry(i,1) transpose(newParticles(:,2))];
        dataStore.particles.theta = [dataStore.particles.theta; dataStore.odometry(i,1) transpose(newParticles(:,3))];
        dataStore.particles.weight = [dataStore.particles.weight; dataStore.odometry(i,1) weights];
    end     
    %pause(0.1);
end

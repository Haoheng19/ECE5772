function [mu_gps, sigma_gps, mu_depth, sigma_depth] = ...
    testEKF(mu_prev, u_prev, sigma_prev, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays)
% testEKF: Performs one step of the extended Kalman Filter, outputs the belief given previous belief
%
%   INPUTS
%       mu_prev           previous vector of pose state (mu(t-1))
%       u_prev           previous command [d; phi]
%       sigma_prev        previous covariance matrix
%       R            state model noise covariance matrix
%       z_gps        current gps measurement vector
%       Q_GPS        GPS measurement noise covariance matrix
%       z_depth      depth measurement vector
%       Q_depth      Realsense depth measurement noise covariance matrix
%       map          map of the environment
%       sensor_pos   sensor position in the robot frame [x y]
%       n_rs_rays    number of evenly distributed realsense depth rays
%       (27...-27) degrees
%
%   OUTPUTS
%       mu_gps      current estimate of vector of pose state (gps)
%       sigma_gps   current covariance matrix (gps)
%       mu_depth    current estimate of vector of pose state (depth)
%       sigma_depth current covariance matrix (depth)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Nusantara, Jonathan

    % you can create pointers to functions in this way:   
    % dynamicsJac   = @(x,u) GjacDiffDrive(x, u); %pointer to jacobian of the dynamics
    % you can create a pointer and send extra parameters in this way:
    % measureJac   = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays); 
    % then, inside your filter, you can call it like this:
    % z = measureJac(mu_prev);
    
    g = @(initPose, d, phi) integrateOdom(initPose, d, phi); % Pointer to integrateOdom
    Gjac = @(x, u) GjacDiffDrive(x, u); % Pointer to GjacDiffDrive
    
    angles = linspace(27*pi/180,-27*pi/180, n_rs_rays); % Calculate angles

    % estimate the state with GPS data using EKF
    h_gps = @(mu_bar) hGPS(mu_bar); % Pointer to hGPS
    Hjac_gps = @(mu_bar) HjacGPS(mu_bar); % Pointer to HjacGPS
    
    % Calling EKF function and passing related functions and variables as parameters
    [mu_gps, sigma_gps] = EKF(mu_prev, u_prev, sigma_prev, z_gps, R, Q, g, Gjac, h_gps, Hjac_gps);
        
    % estimate the state with depth data using EKF
    h_depth = @(mu_bar) depthPredict(mu_bar,map,sensor_pos,angles); % Pointer to depthPredict
    Hjac_depth = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays); % Pointer to Hjac
    
    % Calling EKF function and passing related functions and variables as parameters
    [mu_depth, sigma_depth] = EKF(mu_prev, u_prev, sigma_prev, z_depth, R, Q, g, Gjac, h_depth, Hjac_depth);
    
end
function [mu_gps, sigma_gps, mu_depth, sigma_depth] = ...
    testEKF-ori(mu_prev, u_prev, sigma_prev, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays)
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
    
    g = @(initPose,d,phi) integrateOdom(initPose,d,phi); % Pointer to integrateOdom
    angles = linspace(27*pi/180,-27*pi/180, n_rs_rays);
    
    mu_bar = g(mu_prev, u_prev(1), u_prev(2));
    Gt = GjacDiffDrive(mu_prev, u_prev);
    sigma_bar = Gt * sigma_prev * transpose(Gt) + R;
    
    % estimate the state with GPS data using EKF
    h_gps = hGPS(mu_bar);
    Ht_gps = HjacGPS(mu_bar);
    kt_gps = sigma_bar * transpose(Ht_gps) * inv(Ht_gps * sigma_bar * transpose(Ht_gps) + Q_GPS);
    
    mu_gps = mu_bar + kt_gps * (z_gps - h_gps);
    sigma_gps = (eye(3) - kt_gps * Ht_gps) * sigma_bar;
    
    % estimate the state with depth data using EKF
    Ht_depth = HjacDepth(mu_bar, map, sensor_pos, n_rs_rays);
    kt_depth = sigma_bar * transpose(Ht_depth) * inv(Ht_depth * sigma_bar * transpose(Ht_depth) + Q_depth);
    mu_depth = mu_bar + kt_depth * (transpose(z_depth) - depthPredict(mu_bar,map,sensor_pos,angles));
    sigma_depth = (eye(3) - kt_depth * Ht_depth) * sigma_bar;
    
end
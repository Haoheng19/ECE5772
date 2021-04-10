function [mu_gps, sigma_gps, mu_depth, sigma_depth] = ...
    testEKF(mu_prev, u_prev, sigma_prev, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays)

    dynamics = @(x,u) GjacDiffDrive(x, u);
    measureJac = @(x) HjacDepth(x, map, sensor_pos, n_rs_rays); 
    measureGPS = @(x) hGPS(x);
    JacGPS = @(x) HjacGPS(x);
    angs = linspace((27 * pi / 180), (-27 * pi / 180), n_rs_rays)';
    measureDepth = @(x) depthPredict(x, map, sensor_pos, angs);
    predict = @(mu, u) integrateOdom(mu, u(1), u(2));
    EKF_step = @(mu_prev, u_prev, sigma_prev, z, Q, Hfunc, hfunc)...
        EKF(mu_prev, u_prev, sigma_prev, R, z, Q, predict, dynamics, Hfunc, hfunc);
        
    % estimate the state with GPS data using EKF
    [mu_gps, sigma_gps] = EKF_step(mu_prev, u_prev, sigma_prev, z_gps, Q_GPS, JacGPS, measureGPS);    
    
    % estimate the state with depth data using EKF    
    [mu_depth, sigma_depth] = EKF_step(mu_prev, u_prev, sigma_prev, z_depth, Q_depth, measureJac, measureDepth); 
end
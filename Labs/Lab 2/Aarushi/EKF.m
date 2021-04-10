function [mu, sigma] = EKF(mu_prev, u_prev, sigma_prev, R, z0, Q, predict, dynamics, Hfunc, hfunc)
    
    if length(z0) > 4
        pred_mu = predict(mu_prev, u_prev);
        G = dynamics(mu_prev, u_prev);
        pred_sig = (G * sigma_prev * G') + R;
        H0 = Hfunc(pred_mu);
        h0 = hfunc(pred_mu);
        H = [];
        h = [];
        z = [];
        count = length(h0);
        for i=1:length(z0)
            if (h0(i) >= 10) || (h0(i) <= .175) || (abs(h0(i)-z0(i)) >= .3)
                count = count - 1;
            else
                H = [H; H0(i, :)];
                h = [h; h0(i)];
                z = [z; z0(i)];
            end
        end
        if count <= 3
            mu = pred_mu;
            sigma = pred_sig;
        else
            Q = Q(1:length(z), 1:length(z));
            size(Q)
            k_t = pred_sig * H' * (inv(H * pred_sig * H' + Q));
            mu = pred_mu + k_t * (z - h);
            sigma = ( eye(size(k_t, 1)) - k_t * H ) * pred_sig;
        end
    else
        z = z0;
        pred_mu = predict(mu_prev, u_prev);
        G = dynamics(mu_prev, u_prev);
        pred_sig = (G * sigma_prev * G') + R;
        H = Hfunc(pred_mu);
        h = hfunc(pred_mu);
        k_t = pred_sig * H' * (inv(H * pred_sig * H' + Q));
        mu = pred_mu + k_t * (z - h);
        sigma = ( eye(size(k_t, 1)) - k_t * H ) * pred_sig; 
    end   
end
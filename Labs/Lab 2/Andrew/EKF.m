function [mu, sig] = EKF(mu_prev, u, sig_prev, g, Gjac, R, z, h, Hjac, Q)
   mu_bar = g(mu_prev, u);
   sig_bar = Gjac(mu_prev, u)*sig_prev*Gjac(mu_prev, u)'+R;
   H = Hjac(mu_bar);
   k = sig_bar*H'/(H*sig_bar*H'+Q);
   mu = mu_bar+k*limit(z-h(mu_bar));
   sig = (eye(length(sig_prev))-k*H)*sig_bar;
end

function x = limit(x)
    for i = 1:length(x)
        if x(i) > 3 || x(i) < -3
            x(i) = 0;
        end
    end
end
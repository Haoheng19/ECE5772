function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
    vec = [1 0;0 1/epsilon]*[cos(theta) sin(theta);-sin(theta) cos(theta)]*[cmdVx;cmdVy];
    cmdV = vec(1);
    cmdW = vec(2);
end
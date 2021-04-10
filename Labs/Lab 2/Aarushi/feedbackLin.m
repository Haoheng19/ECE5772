function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)

    espMat = [1,0;0,1/epsilon];
    Rbi = [cos(theta),sin(theta);-1*sin(theta),cos(theta)];
    
    vw = espMat * Rbi * [cmdVx;cmdVy];
    
    cmdV = vw(1,1);
    cmdW = vw(2,1);

end

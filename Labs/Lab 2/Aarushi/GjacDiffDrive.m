function G = GjacDiffDrive(x, u)
    th = x(3);
    d = u(1);
    G = [1 0 -1*d*sin(th); 0 1 d*(cos(th)); 0 0 1];
end

angs = linspace((27 * pi / 180), (-27 * pi / 180), 9)';
depth = @(x) depthPredict(x, [-100000 1 100000 1], [.13 0], angs); 
predict = @(mu, u) integrateOdom(mu, u(1), u(2));

z = ones(9) * 2;
z = z(:,1);
%random points to start
X = (2* rand(30,1)) - 1;
Y = (4* rand(30,1)) - 3;
th = ones(height(Y(:))) * (pi/2);
th = th(:,1);
X_0 = [X, Y, th];
scatter(X_0(:,1), X_0(:,2));
hold on;
R = .001*(eye(3));
Q = .01*(eye(9));
X_t = PF(X_0, [1;0], z, R, Q, depth, predict);
scatter(X_t(:,1), X_t(:,2), '*');
hold on;
plot([-1, 1], [1,1]);
legend({'initial', 'after one time step', 'infinite wall'});
hold off;

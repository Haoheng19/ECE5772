map = [-10000 1 10000 1];
u = [1.0; 0.0]; % Odometry
depth = 2.0 * ones(9,1);
sensorOrigin = [0.13 0];
initialParticle = zeros(30,3);
initialParticle(:,1) = unifrnd(-1,1,1,30);
initialParticle(:,2) = unifrnd(-3,1,1,30);
initialParticle(:,3) = ones(30,1) * pi / 2;

R = 0.01 * ones(3,1);
Q = 0.001 * eye(9);

g = @(initPose, d, phi) integrateOdom(initPose, d, phi);
angles = linspace(27*pi/180,-27*pi/180, 9);
h = @(robotPose) depthPredict(robotPose,map,sensorOrigin,angles);


[newParticles] = PF(initialParticle, u, depth, R, Q, g, h);

scatter(initialParticle(:,1), initialParticle(:,2))
hold on
scatter(newParticles(:,1), newParticles(:,2),'x')
plot([-10 10],[1 1])

title('Particle Filter plot')
legend('initial','after one iteration', 'map')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
%savefig('plot_pf.fig')
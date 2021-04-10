% Plot the trajectory
figure
plot(dataStore02.truthPose(:,2),dataStore02.truthPose(:,3),'-b')
hold on
plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'-r')
scatter([0.88 -0.77 -1.05 0],[-1.99 -2.23 -0.97 0],'g')
legend('Real', 'Simulation', 'Waypoints')
title('Robot trajectory during waypoint [0.88 -1.99; -0.77 -2.23; -1.05 -0.97; 0 0 ] with eps=0.2')
xlabel('x coordinate intertial frame') 
ylabel('y coordinate intertial frame')
savefig('plot_trajectory_24e.fig')
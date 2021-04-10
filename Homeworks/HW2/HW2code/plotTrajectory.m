no_noise = integrateOdom([0 0 0],transpose(odom_no(:,2)),transpose(odom_no(:,3)));
noise = integrateOdom([0 0 0],odom_noise2(:,2),odom_noise2(:,3));
% gaussian noise std=0.5

%no_noise(2,:);
transpose(odom_noise2(:,2));

plot(trueTrajectory(:,2),trueTrajectory(:,3),'b-')
hold on
plot(no_noise(1,:),no_noise(2,:),'r-')
hold on
plot(noise(1,:),noise(2,:),'g-')
hold off

title('Trajectories of different runs')
legend('overhead localization','noiseless odom','noise odom')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
savefig('plot_trajectory_noises1.fig')
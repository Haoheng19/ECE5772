%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Nusantara, Jonathan

pose1 = [3 -4 2*pi/3];
pose2 = [0 3 pi/2];
angular = 120*pi / 180;
data_xy = lidar_range2xy(lidarScan, 0.2, [-angular angular]);
temp1 = zeros(681,2);
temp2 = zeros(681,2);

for i = 1 : 681
    temp1(i,1:2) = robot2global(pose1,[data_xy(1,i) data_xy(2,i)]);
    temp2(i,1:2) = robot2global(pose2,[data_xy(1,i) data_xy(2,i)]);
end

% Plot
scatter(temp1(:,1), temp1(:,2));
hold on
scatter(temp2(:,1), temp2(:,2));
hold off
legend('pose = [3 -4 2*pi/3]','pose = [0 3 pi/2]')
title('Lidar scan points in global reference')
xlabel('x coordinate')
ylabel('y coordinate')
savefig('plot_lidar.fig')
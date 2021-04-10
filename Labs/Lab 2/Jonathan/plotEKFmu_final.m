
hold on

p2 = plot(dataStore.deadReck(:,2), dataStore.deadReck(:,3),'g')
p1 = plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3),'r')
p3 = plot(dataStore.ekfMu(:,2), dataStore.ekfMu(:,3),'b')

hold on

for n=1:10
    %line(HW3map([HW3map(n,1) HW3map(n,3)], [HW3map(n,2) HW3map(n,4)]))
    hold on
    plot([lab2WallMap(n,1), lab2WallMap(n,3)], [lab2WallMap(n,2), lab2WallMap(n,4)])
end


title('EKF with GPS mu plot')
legend([p1 p2 p3],{'Truth Pose','Dead Reckoning', 'EKF mu'})
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
% savefig('plot_ekf_depth_mu_test.fig')
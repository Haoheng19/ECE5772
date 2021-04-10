

for n=1:10
    %line(HW3map([HW3map(n,1) HW3map(n,3)], [HW3map(n,2) HW3map(n,4)]))
    hold on
    plot([lab2WallMap(n,1), lab2WallMap(n,3)], [lab2WallMap(n,2), lab2WallMap(n,4)],'k')
end

sigma = dataStore.ekfSigma(:,1:2)

temp = reshape(dataStore.ekfSigma(1,2:end),3,3)
p1 = plotCovEllipse(dataStore.ekfMu(1,2:3),temp(1:2,1:2),1,[{'color'},{'r'}])
for i = 6:size(dataStore.ekfMu(:,2:3),1)/5
    temp = reshape(dataStore.ekfSigma(i*5,2:end),3,3)
    p2 = plotCovEllipse(dataStore.ekfMu(i*5,2:3),temp(1:2,1:2),1,[{'color'},{'b'}])
end
temp = reshape(dataStore.ekfSigma(size(dataStore.ekfMu(:,2:3),1),2:end),3,3)
p3 = plotCovEllipse(dataStore.ekfMu(size(dataStore.ekfMu(:,2:3),1),2:3),temp(1:2,1:2),1,[{'color'},{'g'}])


title('EKF with GPS sigma plot')
legend([p1 p2 p3], {'Initial time', 'Other times', 'Final time'})
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
savefig('plot_ekf_gps_sigma_diffInit.fig')
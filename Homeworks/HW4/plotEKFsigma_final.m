

% for n=1:12
%     %line(HW3map([HW3map(n,1) HW3map(n,3)], [HW3map(n,2) HW3map(n,4)]))
%     hold on
%     plot([cornerMap(n,1), cornerMap(n,3)], [cornerMap(n,2), cornerMap(n,4)])
% end
sigma = dataStore.ekfSigma(:,1:2)

for i = 1:size(dataStore.ekfMu(:,2:3),1)
    temp = reshape(dataStore.ekfSigma(i,2:end),3,3)
    plotCovEllipse(dataStore.ekfMu(i,2:3),temp(1:2,1:2))
end


title('EKF with depth sigma plot')
%legend('Truth Pose','Dead Reckoning', 'EKF mu')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
savefig('plot_ekf_depth_sigma_2.fig')
sensordata = zeros(length(dataStore.truthPose),10)

for j = 1:length(dataStore.truthPose)
    [depth] = depthPredict(dataStore.truthPose(j,2:4),[0 2 2 0; 2 0 0 -2; -2 0 0 -2; -2 0 0 2],[0.16 0],linspace((27*pi/180),-(27*pi/180),9));
    %hold on
    sensordata(j,2:10) = depth;
    sensordata(j,1) = dataStore.truthPose(j,1);
end

for i = 3:11
    hold on
    plot(dataStore.rsdepth(:,1),dataStore.rsdepth(:,i),'-b')
    plot(sensordata(:,1),sensordata(:,i-1),'-r')
end

title('Depth measurement comparison')
legend('rsdepth','calculated from pose')
xlabel('Time in seconds') 
ylabel('Depth measured in meters')
savefig('plot_depth_compare3.fig')
hold on

% % Initial particle
% p1 = scatter(dataStore.particles.x(1,2:end), dataStore.particles.y(1,2:end),'ro');
% 
% for k = 2:size(dataStore.particles.x ,2)
%         line([dataStore.particles.x(1,k), dataStore.particles.x(1,k)+0.1*cos(dataStore.particles.theta(1,k))], [dataStore.particles.y(1,k), dataStore.particles.y(1,k)+0.1*sin(dataStore.particles.theta(1,k))],'Color','blue');
% end

% highestWeight = []
% 
% for i = 1:size(dataStore.particles.x ,1)
%     currentWeights = dataStore.particles.weight(i,2:21);
%     % max
%     [temp, index] = max(currentWeights);
%     highestWeight = [highestWeight; dataStore.particles.x(i,index+1) dataStore.particles.y(i,index+1)];
    %p4 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'bx');
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         p5 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'gx');
%     end
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         p6 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'bx');
%     end
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         p7 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'cx');
%     end
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         p8 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'mx');
%     end
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         p9 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'yx');
%     end
% end

% Final particle
% p2 = scatter(dataStore.particles.x(end,2:end), dataStore.particles.y(end,2:end),'g^');

% True pose
% p3 = plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3),'k');

%p4 = plot(highestWeight(:,1),highestWeight(:,2),'b');



% Plotting comparison of averaging


% True pose
plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3),'g')

highest = [];
average = [];

for i = 1:size(dataStore.particles.x ,1)
    currentWeights = dataStore.particles.weight(i,2:end);
    currentWeights5000 = dataStoremany.particles.weight(i,2:end);
    % max
    [temp, index] = max(currentWeights);
    [temp, index5000] = max(currentWeights5000);
    %scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'^')
    
    highest = [highest; dataStore.particles.x(i,index+1) dataStore.particles.y(i,index+1)];
    highest5000 = [highest5000; dataStoremany.particles.x(i,index5000+1) dataStoremany.particles.y(i,index5000+1)];
%     cum_x = dataStore.particles.x(i,index+1);
%     cum_y = dataStore.particles.y(i,index+1);
%     
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         cum_x = cum_x + dataStore.particles.x(i,index+1);
%         cum_y = cum_y + dataStore.particles.y(i,index+1);
%     end
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         cum_x = cum_x + dataStore.particles.x(i,index+1);
%         cum_y = cum_y + dataStore.particles.y(i,index+1);
%     end
    average = [average; cum_x/3 cum_y/3];
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         count3 = count3 + 1;
%     end
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         count4 = count4 + 1;
%     end
%     if ~isempty(temp)
%         [temp, index] = max(currentWeights(currentWeights<temp));
%         count5 = count5 + 1;
        %scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'>')
      %end
end

plot(highest(:,1), highest(:,2),'r')
plot(highest5000(:,1), highest5000(:,2),'b')
%plot(average(:,1), average(:,2),'b')

for n=1:10
    %line(HW3map([HW3map(n,1) HW3map(n,3)], [HW3map(n,2) HW3map(n,4)]))
    hold on
    plot([lab2WallMap(n,1), lab2WallMap(n,3)], [lab2WallMap(n,2), lab2WallMap(n,4)])
end


title('PF with depth plot 50vs5000 particles')
% legend([p1 p2 p3], {'Initial particles', 'Final Particles', 'Truth pose trajectory'})
legend('Truth pose trajectory','50particles highest weight trajectory', '5000particles highest weight trajectory')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
savefig('plot_PF_depth_50vs5000.fig')
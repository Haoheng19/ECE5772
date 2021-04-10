hold on

% Initial particle
p1 = scatter(dataStore.particles.x(1,2:end), dataStore.particles.y(1,2:end),'bo');

for k = 2:size(dataStore.particles.x ,2)
        line([dataStore.particles.x(1,k), dataStore.particles.x(1,k)+0.1*cos(dataStore.particles.theta(1,k))], [dataStore.particles.y(1,k), dataStore.particles.y(1,k)+0.1*sin(dataStore.particles.theta(1,k))],'Color','blue');
end


for i = 1:size(dataStore.particles.x ,1)
    currentWeights = dataStore.particles.weight(i,2:21);
    % max
    [temp, index] = max(currentWeights);
    p4 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'rx');
    if ~isempty(temp)
        [temp, index] = max(currentWeights(currentWeights<temp));
        p5 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'gx');
    end
    if ~isempty(temp)
        [temp, index] = max(currentWeights(currentWeights<temp));
        p6 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'bx');
    end
    if ~isempty(temp)
        [temp, index] = max(currentWeights(currentWeights<temp));
        p7 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'cx');
    end
    if ~isempty(temp)
        [temp, index] = max(currentWeights(currentWeights<temp));
        p8 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'mx');
    end
    if ~isempty(temp)
        [temp, index] = max(currentWeights(currentWeights<temp));
        p9 = scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'yx');
    end
end

% Final particle
p2 = scatter(dataStore.particles.x(end,2:end), dataStore.particles.y(end,2:end),'k^');

% True pose
p3 = plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3),'k');




% Plotting comparison of averaging

% 
% % True pose
% plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3),'g')
% 
% highest = [];
% average = [];
% 
% for i = 2:size(dataStore.particles.x ,1)
%     currentWeights = dataStore.particles.weight(i,2:21);
%     % max
%     [temp, index] = max(currentWeights);
%     %scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'^')
%     
%     highest = [highest; dataStore.particles.x(i,index+1) dataStore.particles.y(i,index+1)];
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
%     average = [average; cum_x/3 cum_y/3];
% %     if ~isempty(temp)
% %         [temp, index] = max(currentWeights(currentWeights<temp));
% %         count3 = count3 + 1;
% %     end
% %     if ~isempty(temp)
% %         [temp, index] = max(currentWeights(currentWeights<temp));
% %         count4 = count4 + 1;
% %     end
% %     if ~isempty(temp)
% %         [temp, index] = max(currentWeights(currentWeights<temp));
% %         count5 = count5 + 1;
%         %scatter(dataStore.particles.x(i,index+1), dataStore.particles.y(i,index+1),'>')
%       %end
% end
% 
% plot(highest(:,1), highest(:,2),'r')
% plot(average(:,1), average(:,2),'b')

for n=1:12
    %line(HW3map([HW3map(n,1) HW3map(n,3)], [HW3map(n,2) HW3map(n,4)]))
    hold on
    plot([cornerMap(n,1), cornerMap(n,3)], [cornerMap(n,2), cornerMap(n,4)])
end


title('PF with depth plot 500 particles')
legend([p1 p2 p3 p4 p5 p6 p7 p8 p9], {'Initial pose predictions','Final pose predictions', 'Truth pose trajectory', '1st highest weight predictions', '2nd highest weight predictions', '3rd highest weight predictions', '4th highest weight predictions', '5th highest weight predictions', '6th highest weight predictions'})
% legend('Truth pose trajectory','Highest weight trajectory', 'Top 3 weights trajectory')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
savefig('plot_PF_depth_500.fig')
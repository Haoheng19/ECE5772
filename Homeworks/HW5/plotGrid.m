bumpOffsetx = 0.16 * sind(45);
bumpOffsety = 0.16 * cosd(45);

%load('loopMap.mat')
for n=1:15
    hold on
    plot([loopMap(n,1), loopMap(n,3)], [loopMap(n,2), loopMap(n,4)],'g')
end

p1 = plot(dataStore.truthPose(:,2), dataStore.truthPose(:,3),'r')

xTicks = linspace(-2.5,2.5,26)
yTicks = linspace(-2.5,2.5,26)

for i = 1:26
    plot([xTicks(i),xTicks(i)],[-2.5,2.5],'k')
    plot([-2.5,2.5],[xTicks(i),xTicks(i)],'k')
end

for i = 1:length(dataStore.truthPose)
    bump = dataStore.bump(i, 2) || dataStore.bump(i, 3) || dataStore.bump(i, 7);
    if bump == 1
        p2 = scatter(dataStore.truthPose(i,2), dataStore.truthPose(i,3),'b^');
%     if dataStore.bump(i, 2) == 1 % bumpRight
%         bumpLocation = robot2global(dataStore.truthPose(i,2:4),[bumpOffsetx -bumpOffsety]);
%         p2 = scatter(bumpLocation(1), bumpLocation(2),'b^');
%     elseif dataStore.bump(i, 3) == 1 % bumpLeft
%         bumpLocation = robot2global(dataStore.truthPose(i,2:4),[bumpOffsetx bumpOffsety]);
%         p2 = scatter(bumpLocation(1), bumpLocation(2),'b^');
%     elseif dataStore.bump(i, 7) == 1 % bumpFront
%         bumpLocation = robot2global(dataStore.truthPose(i,2:4),[0.16 0]);
%         p2 = scatter(bumpLocation(1), bumpLocation(2),'b^');
    end
end



legend([p1 p2],{'Truth Pose', 'Bump Location'})
title('Map with Grids, Trajectory, and Bumps plot')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
savefig('plotGridPoseBump.fig')
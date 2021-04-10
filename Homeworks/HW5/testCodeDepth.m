grid = logOddsDepth(dataStore.truthPose(:,2:4), dataStore.rsdepth(:,3:11), 0, [25 25], [-2.5 2.5 -2.5 2.5])

tempx = linspace(-2.4,2.4,25);
tempy = linspace(-2.4,2.4,25);
% 
% grid = logOddsDepth(dataStore.truthPose(:,2:4), dataStore.rsdepth(:,3:11), 0, [50 50], [-2.5 2.5 -2.5 2.5])
% 
% tempx = linspace(-2.45,2.45,50);
% tempy = linspace(-2.45,2.45,50);

[meshX,meshY] = meshgrid(tempx,tempy);


meshX = reshape(meshX,1,[]);
meshY = reshape(meshY,1,[]);

plotOccupancyGrid(meshX, meshY, grid);

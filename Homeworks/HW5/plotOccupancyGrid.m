function plotOccupancyGrid(coodinatesXOfGrid, coodinatesYOfGrid, logodds)
% plotGridBelief: plot a pdf 
%                          
%   INPUTS
%       coodinatesXOfGrid   x coorindates of grid centers 
%       coodinatesYOfGrid   y coorindates of grid centers
%       logodds             log odds of each cell in the occupancy grid

% OPTION 1: Rounding the log odds values to -1 to 8 range
logodds(logodds>7.5) = 8;
logodds(logodds<-0.736) = -1;
% normalize to 0-1
logodds = (logodds - min(min(logodds))) / (max(max(logodds)) - min(min(logodds)));

% OPTION 2: Rounding the log odds values to binary 0 or 1
% logodds(logodds>=-0.5) = 1;
% logodds(logodds<-0.5) = 0;

% plot the belief
image(coodinatesXOfGrid,coodinatesYOfGrid,logodds,'CDataMapping','scaled'); 
grid = colormap (flipud(gray))
%title(grid, '25x25 Occupancy grid based on depth sensor, full trajectory')
xlabel('X intertial coordinate')
ylabel('Y inertial coordinate')

bar = colorbar
ylabel(bar, 'Range of confidence of free vs occupied cell')
caxis([min(min(logodds)) max(max(logodds))])

% flip axis (to make y point up)
ax = gca;
ax.YDir = 'normal';

%savefig('plotDepth25-full.fig')
end
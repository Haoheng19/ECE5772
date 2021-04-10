% 
% for n=1:11
%     %line(HW3map([HW3map(n,1) HW3map(n,3)], [HW3map(n,2) HW3map(n,4)]))
%     hold on
%     plot([HW3map(n,1), HW3map(n,3)], [HW3map(n,2), HW3map(n,4)])
% end
% 
% max(max(max(HW3map(:,1))), max(max(HW3map(:,3))))
% max(max(max(HW3map(:,2))), max(max(HW3map(:,4))))
% min(min(min(HW3map(:,1))), min(min(HW3map(:,3))))
% min(min(min(HW3map(:,2))), min(min(HW3map(:,4))))

%a = ones(1,30) * 3

%a(30) = 5

%length(stationary)

% 
% x = [-2,-1,0,1,2];
% mu = 0;
% sigma = 0.3;
% y = normpdf(x,mu,sigma)

% stationary(1,2)
% if isnan(stationary(1,2)) == 1
%     a=1+1
% end

grid_size = [40 22];

[a] = gridLocalizationStationary(HW3map, stationary, grid_size)
sum(sum(a))
size(a)


maximum = max(max(a));
index = find(a==maximum);



map_min_x = min(min(min(HW3map(:,1))), min(min(HW3map(:,3))));
map_min_y = min(min(min(HW3map(:,2))), min(min(HW3map(:,4))));
map_max_x = max(max(max(HW3map(:,1))), max(max(HW3map(:,3))));
map_max_y = max(max(max(HW3map(:,2))), max(max(HW3map(:,4))));
map_width = map_max_x - map_min_x;
map_height = map_max_y - map_min_y;
cell_width = map_width / grid_size(1);
cell_height = map_height / grid_size(2);


cell_col = mod(index,grid_size(1)); % x axis
cell_row = ceil(index / grid_size(1)); % y axis
pose_x = (cell_col * cell_width) - (cell_width / 2);
pose_y = (cell_row * cell_height) - (cell_height / 2);


% c = linspace((1 * cell_width) - (cell_width / 2),(10 * cell_width) - (cell_width / 2),10);
% f = [c c c c c c c c c c];
% d = linspace((1 * cell_height) - (cell_height / 2),(10 * cell_height) - (cell_height / 2),10);
% g = [ones(1,10)*0.55 ones(1,10)*1.65 ones(1,10)*2.75 ones(1,10)*3.85 ones(1,10)*4.95 ones(1,10)*6.05 ones(1,10)*7.15 ones(1,10)*8.2500 ones(1,10)*9.3500 ones(1,10)*10.4500];

f = []
g = []

for ky = 1:grid_size(2) % Across all y
    for kx = 1:grid_size(1) % Across all x
        f = [f kx];
        g = [g ky];
    end
end

f = f * cell_width - (cell_width / 2);
g = (g * cell_height) - (cell_height / 2);
h = reshape(ones(1,100),10,10) * 0.01;
hold on
plotGridBelief(f, g, a)
%plotGridBelief(f, g, h)
title('Initial PDF plot on map with grid 10x10')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
%savefig('plot_grid_1010_initial_.fig')

%scatter(f,g)

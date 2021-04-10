% valid = ~isnan(stationary(1,:));
% 
% C_init = [0 -1; -1 0; 0 1; 1 0];
% sensor_noise = [0.1 0.3 0.1 0.3];
% depth = [1 2 3 4]
% 
% C = [];
% Q_temp = [];
% exp_measurement = [];
% z_t = []
% for i = 1:4 % For the number of directions
%     if valid(i) == 1 % If a measurement is received for that direction
%         C = [C; C_init(i,:)]; % Add that C entry to temporary C
%         Q_temp = [Q_temp sensor_noise(i)];
%         exp_measurement = [exp_measurement; depth(i)];
%         z_t = [z_t; stationary(1,i)];
%     end
% end
% 
% 
% C
% Q = diag(Q_temp)
% exp_measurement
% z_t

[a, b] = KFStationary(HW3map, stationary, [10; 5.5], [(5/3)^2 0; 0 (2/3)^2])

% plotCovEllipse([14; 5.5],[(5/3)^2 0; 0 (2/3)^2])
% hold on
% plotCovEllipse(a,b)
% title('Pos and Covariance estimate with init mu=[14; 5.5 var=[(5/3)^2 0; 0 (2/3)^2]')
% xlabel('X intertial coordinate') 
% ylabel('Y intertial coordinate')
% legend('Initial time','Final time')
% savefig('plot_kf_2.fig')
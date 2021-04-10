head = .1;
num_part = 20;

data = dataStore;

%averages
% x = data.tier5;
% for i=1:(length(data.tier5)/5)
%     xn = [xn; [sum(x((i-1)*5 + 1:i*5,1)), sum(x((i-1)*5 + 1:i*5,2)), sum(x((i-1)*5 + 1:i*5,3)) ] / 5 ];    
% end


%plot truthPose
plot(data.truthPose(:,2), data.truthPose(:,3), 'r')
hold on

i = 1;
%plot one of everything
init = data.particles(1:num_part,:);
plot(init(i,1), init(i,2), 'go');
x = data.tier5;
plot(x(i,1), x(i,2), 'co');
% plot(xn(i,1), xn(i,2), 'bo');
x = data.big1;
plot(x(i,1), x(i,2), 'mo');
final = data.particles(end-num_part+1:end,:);
plot(final(i,1), final(i,2), 'ko');


%plot inital
init = data.particles(1:num_part,:);
for i= 1:length(init)
    plot(init(i,1), init(i,2), 'go');
    plot([init(i,1), init(i,1) + head*cos(init(i,3))], ...
        [init(i,2), init(i,2) + head*sin(init(i,3))], 'g');
    hold on
end

%plot next big 5
x = data.tier5;
for i=1:length(x)
    plot(x(i,1), x(i,2), 'co');
    hold on
end

%plot avgs
% for i=1:length(xn)
%     plot(xn(i,1), xn(i,2), 'bo');
%     hold on
% end

%plot trajectory of heighest weight
x = data.big1;
for i=1:length(x)
    plot(x(i,1), x(i,2), 'mo');
    hold on
end

%plot final
final = data.particles(end-num_part+1:end,:);
for i=1:length(final)
    plot(final(i,1), final(i,2), 'ko');
    hold on
end


helper

xlabel('X')
ylabel('Y')
title('Particle Filter, 20 intial')

legend({'truthPose', 'X_0', 'X_t top5', 'X_t largest', 'X_f'})

hold off
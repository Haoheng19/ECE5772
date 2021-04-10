plot(dataStore03(:,2),dataStore03(:,3),'b-')
hold on
plot(dataStore05(:,2),dataStore05(:,3),'r-')
hold on
plot(dataStore08(:,2),dataStore08(:,3),'g-')
hold off

title('Trajectory in Intertial with eps = 0.3, 0.5, 0.8')
legend('eps = 0.3','eps = 0.5','eps = 0.8')
xlabel('X intertial coordinate') 
ylabel('Y intertial coordinate')
savefig('plot_trajectory_local.fig')
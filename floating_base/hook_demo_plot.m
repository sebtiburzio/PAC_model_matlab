%%
clear
clear global

load('../object_parameters/full_dynamic_id/black_short_loop_100g.mat')
state_evolution = readtable("./data_in/0402-loop_demo_static_id/650_0/state_evolution.csv");
measurements = readtable("./data_in/0402-loop_demo_static_id/650_0/measurements.csv");
sequence = readtable("./data_in/0402-loop_demo_static_id/650_0/sequence.csv");

%% Base Plot with line references
yyaxis left
plot(state_evolution.ts,-state_evolution.Phi)
hold on
xlabel('t (s)','Interpreter','latex')
ylabel('$\phi$ (rad)','Interpreter','latex')
yyaxis right
plot(state_evolution.ts,state_evolution.X)
plot(state_evolution.ts,state_evolution.Z)
ylim([0.1,1.4])
xlim([0 32])
ylabel('x , y (m)','Interpreter','latex')
legend('$\phi$','x ','y','Interpreter','latex','Location','northwest','Orientation','vertical')
ax = gca;
set(ax, 'FontSize', 30)
set(ax, 'TickLabelInterpreter', 'latex')
lines = findobj(gcf, 'Type', 'line');
for i = 1:length(lines)
    set(lines(i), 'markersize', 10, 'linewidth', 2);
end
grid on
box on

%% Base Plot with line references
% yyaxis left
% plot(state_evolution.ts,-state_evolution.Phi,'LineStyle','-')
% hold on
% stairs(sequence_ts,[-sequence.Phi;-sequence.Phi(end)],'LineStyle','--','LineWidth',1.5)
% xlabel('t (s)','Interpreter','latex')
% ylabel('$\phi$ (rad)','Interpreter','latex')
% yyaxis right
% plot(state_evolution.ts,state_evolution.X,'LineStyle','-')
% plot(state_evolution.ts,state_evolution.Z,'LineStyle','-.')
% stairs(sequence_ts,[sequence.X;sequence.X(end)],'LineStyle',':','LineWidth',1.5)
% stairs(sequence_ts,[sequence.Z;sequence.Z(end)],'LineStyle','--','LineWidth',1.5)
% ylim([0.1,1.4])
% xlim([0 32])
% ylabel('x , y (m)','Interpreter','latex')
% legend('$\phi$','$\phi^*$','x ','y','$x^*$','$y^*$','Interpreter','latex','Location','southeast','Orientation','horizontal')
% ax = gca;
% set(ax, 'FontSize', 30)
% set(ax, 'TickLabelInterpreter', 'latex')
% lines = findobj(gcf, 'Type', 'line');
% for i = 1:length(lines)
%     set(lines(i), 'markersize', 10, 'linewidth', 2);
% end
% grid on
% box on

%%
plot(measurements.ts,[measurements.X_end,measurements.Z_end])
hold on
sequence_ts = [14.666,18.333,21.333,24.666,measurements.ts(end)];
plot(sequence_ts,[sequence.GoalX(1),sequence.GoalX(3),sequence.GoalX(3:5)'],'LineStyle','--','LineWidth',1.5,'Color',[0 0.4470 0.7410]) % A bit of manual 
plot(sequence_ts,sequence.GoalZ,'LineStyle','--','LineWidth',1.5,'Color',[0.8500 0.3250 0.0980])
% sequence_ts = [0,14.666,18.333,21.333,24.666,measurements.ts(end)];
% stairs(sequence_ts,[sequence.GoalX(1);sequence.GoalX(1);sequence.GoalX(3);sequence.GoalX(5);sequence.GoalX(5);sequence.GoalX(5)],'LineStyle','--','LineWidth',1.5,'Color',[0 0.4470 0.7410])
% stairs(sequence_ts,[sequence.GoalZ(1);sequence.GoalZ(1);sequence.GoalZ(3);sequence.GoalZ(5);sequence.GoalZ(5);sequence.GoalZ(5)],'LineStyle','--','LineWidth',1.5,'Color',[0.8500 0.3250 0.0980])
xlabel('t (s)','Interpreter','latex')
ylabel('x , y (m)','Interpreter','latex')
ylim([0.1,0.8])
xlim([0 32])
legend('$p_{e,x}$','$p_{e,y}$','$p^*_x$','$p^*_y$','Interpreter','latex','Location','northwest','Orientation','vertical')
ax = gca;
set(ax, 'FontSize', 30)
set(ax, 'TickLabelInterpreter', 'latex')
lines = findobj(gcf, 'Type', 'line');
for i = 1:length(lines)
    set(lines(i), 'markersize', 10, 'linewidth', 2);
end
grid on
box on

%%
plot(state_evolution.ts,[state_evolution.Theta0,state_evolution.Theta1])
hold on
stairs(sequence_ts,[sequence.Theta0;sequence.Theta0(end)])
stairs(sequence_ts,[sequence.Theta1;sequence.Theta1(end)])
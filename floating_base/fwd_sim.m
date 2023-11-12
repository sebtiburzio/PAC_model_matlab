%% Init
clear
clear global
rmpath('../fixed_base/automatically_generated')
addpath('automatically_generated')
global k_obj K p_vals Theta_bar
global beta_obj D

%%
% Load predefined object parameters
load('../object_parameters/black_unweighted.mat')
% load('../object_parameters/full_dynamic_id/black_weighted_dyn.mat')

%%
% % Manually defined object parameters (overwrites loaded parameters)
% % Static properties
% p_vals = [0.6, 0.23, 0.6, 0.02]';
% k_obj = 0.1965;
% Theta_bar = [0.1421; 1.2391];
% 
% % Dynamic properties
% beta_obj = 0.0547;

%%
k_base = diag([0 1e3 0]);
beta_base = diag([1e-2 1e-2 1e-2]);
Theta_bar = [Theta_bar(1); Theta_bar(2); 0; 0; 0];
H = [1, 1/2; 1/2, 1/3];
K = [k_obj*H     zeros(2,3);
     zeros(3,2)  k_base];
D = [beta_obj*H  zeros(2,3);
     zeros(3,2)  beta_base];

% Simulation set up
global G_Scale G_dir
G_Scale = 1.0;
G_dir = 0.0;
% Initial condition
x_0 = [-0.34; 2.14; 0; 0; -0.06];
dx_0 = [0; 0; 0; 0; 0];

%% For comparing to imported measured dynamic evolution
% Import the full state and input data csv as column vectors first
% Measured data is force/torque reading. Actuation is negative of this
% Also additional moment due to offset from FT sensor and cable base (only set manually for Phi=0 case for now. 0.0485m is the offset for the black cables)
actuation = [ts -Fx -Fz -Ty-Fx*0.0485]; % Simulink model reads this matrix (when connected as input)
x_0 = [Theta0(1);Theta1(1);X(1);Z(1);Phi(1)];
dx_0 = [dTheta0(1);dTheta1(1);dX(1);dZ(1);dPhi(1)];
mdl = 'dynamics';
open_system(mdl)
set_param(mdl,"StopTime",string(ts(end)))

%% Call simulink
out = sim('dynamics');
q_ev = out.q_ev; % or if sim doesn't finish: q_ev = load('q_ev.mat');

%% Plot/compare evolutions - Curvature
plot(ts,[Theta0 Theta1])
hold on
plot(q_ev.Time,[q_ev.Data(:,1) q_ev.Data(:,2)])
hold off
xlabel('t (s)','Interpreter','latex')
ylabel('$\theta$ (rad)','Interpreter','latex')
xlim([0 ts(end)])
ylim([-2.5 5])
legend('$\theta_0$ (measured)','$\theta_1$ (measured)','$\theta_0$ (simulated)','$\theta_1$ (simulated)','Interpreter','latex')
ax = gca;
set(ax, 'FontSize', 30)
set(ax, 'TickLabelInterpreter', 'latex')
lines = findobj(gcf, 'Type', 'line');
for i = 1:length(lines)
    set(lines(i), 'markersize', 10, 'linewidth', 2);
end
grid on
box on
% set(gcf, 'Position', [303 495 1115 331])

%% Plot/compare evolutions - Base coordinates
figure
yyaxis left
plot(ts,[X Z])
hold on
plot(q_ev.Time,[q_ev.Data(:,3) q_ev.Data(:,4)])
ylabel('x , y (m)','Interpreter','latex')
yyaxis right
plot(ts,Phi)
plot(q_ev.Time,q_ev.Data(:,5))
xlabel('t (s)','Interpreter','latex')
ylabel('$\phi$ (rad)','Interpreter','latex')
xlim([0 ts(end)])
legend('x (measured)','y (measured)','x (simulated)','y (simulated)','$\phi$ (measured)','$\phi$ (simulated)' ...
    ,'Interpreter','latex','Location','southeast')
ax = gca;
set(ax, 'FontSize', 30)
set(ax, 'TickLabelInterpreter', 'latex')
lines = findobj(gcf, 'Type', 'line');
for i = 1:length(lines)
    set(lines(i), 'markersize', 10, 'linewidth', 2);
end
grid on
box on
% set(gcf, 'Position', [303 495 1115 331])

%% Save output
% n_step = q_ev.time(end)*30;
% stop_time = q_ev.time(end);
% time_vect = linspace(0,stop_time,n_step);
% q_ev_res = resample(q_ev,time_vect);
% data = [time_vect', q_ev_res.Data(:,1), q_ev_res.Data(:,2)];
% writematrix(data,'black_swing_sim_newMdef_k2_b015_G40_off_n70_220_long.csv');

%% Check output
figure
plot(q_ev.Time(:),q_ev.Data(:,3))
title('X')
figure
plot(q_ev.Time(:),q_ev.Data(:,8))
title('v_X')
figure
plot(q_ev.Time(:),q_ev.Data(:,9))
title('v_Z')
figure
plot(q_ev.Time(:),q_ev.Data(:,10))
title('v_Phi')
figure
plot(q_ev.Time(:),q_ev.Data(:,13))
title('a_X')
figure
plot(q_ev.Time(:),q_ev.Data(:,14))
title('a_Z')
figure
plot(q_ev.Time(:),q_ev.Data(:,15))
title('a_Phi')
figure
plot(q_ev.Time(1:end-1),diff(q_ev.Data(:,13))/diff(q_ev.Time(1:end)))
title('j_X')
figure
plot(q_ev.Time(1:end-1),diff(q_ev.Data(:,14))/diff(q_ev.Time(1:end)))
title('j_Z')
figure
plot(q_ev.Time(1:end-1),diff(q_ev.Data(:,15))/diff(q_ev.Time(1:end)))
title('j_Phi')

%% Save output for robot execution at 1000Hz
q_ev_res = resample(q_ev,linspace(0,q_ev.Time(end),q_ev.Time(end)/0.001));
save('q_ev');
save('q_ev_res');
t_start = 5000;
t_end = 20000;
writematrix([q_ev_res.Data(t_start:t_end,3), q_ev_res.Data(t_start:t_end,4), q_ev_res.Data(t_start:t_end,5)],'trajectory.csv','Delimiter','tab')
writematrix([q_ev_res.Data(t_start:t_end,8), q_ev_res.Data(t_start:t_end,9), q_ev_res.Data(t_start:t_end,10)],'trajectory_vel.csv','Delimiter','tab')

%% Init
clear
addpath('automatically_generated')

% Object properties
% Object properties
global p_vals
p_vals = [0.4, 0.23, 0.75, 0.15];

global K Theta_bar D
k_obj = 5e-2;
beta_obj = 2.5e-2;
k_base = diag([0 0 0 ]);
beta_base = diag([1e-1 1e-1 1e-1]);
Theta_bar = [-0.1104; 0.1523; 0; 0; 0];
H = [1, 1/2; 1/2, 1/3];
K = [k_obj*H     zeros(2,3);
     zeros(3,2)  k_base];
D = [beta_obj*H  zeros(2,3);
     zeros(3,2)  beta_base];

global G_Scale B_Scale
G_Scale = 1.0;
B_Scale = 1.0;

% Put initial condition here
x_0 = [1e-3; 1e-3; 0; 0; 0];
dx_0 = [0; 0; 0; 0; 0];

%% Call simulink
out = sim('dynamics');
q_ev = out.q_ev; % or if sim doesn't finish: q_ev = load('q_ev.mat');
%plot_robot(q_ev)

%% Plot Theta evolution
% plot(ts,Theta0)
% hold on
% plot(q_ev.Time,q_ev.Data(:,1))
% plot(ts,Theta1)
% plot(q_ev.Time,q_ev.Data(:,2))

%% Save output
% n_step = q_ev.time(end)*30;
% stop_time = q_ev.time(end);
% time_vect = linspace(0,stop_time,n_step);
% q_ev_res = resample(q_ev,time_vect);
% data = [time_vect', q_ev_res.Data(:,1), q_ev_res.Data(:,2)];
% writematrix(data,'black_swing_sim_newMdef_k2_b015_G40_off_n70_220_long.csv');

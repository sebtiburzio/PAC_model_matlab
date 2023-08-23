%% Init
clear
addpath('automatically_generated')
global k_obj K p_vals Theta_bar
global beta_obj D

%%
% Load predefined object parameters
load('../object_parameters/orange_short_unweighted.mat')

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
k_base = diag([0 0 0]);
beta_base = diag([1e-1 1e-1 1e-1]);
Theta_bar = [Theta_bar(1); Theta_bar(2) 0; 0; 0];
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
x_0 = [Theta0(1); Theta1(1)];
dx_0 = [dTheta0(1); dTheta1(1)];

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

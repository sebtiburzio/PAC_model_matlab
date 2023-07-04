%% Init
clear
addpath('automatically_generated/fixed')

%%
% Object properties
global p_vals G_dir
p_vals = [0.6, 0.23, 0.61, 0.2];
G_dir = pi/2;

k_obj = 2.0e-1;
beta_obj = 3.0e-2;

global K Theta_bar D
global G_Scale BC_Scale
G_Scale = 1.0;
BC_Scale = 2.0;

H = [1, 1/2; 1/2, 1/3];
K = k_obj*H;
Theta_bar = [0;0];%[-0.5585; 1.9685];
D = beta_obj*H;

% Put initial condition here
x_0 = [0.8398, 0.9675];
dx_0 = [0; 0];

%% Call simulink
out = sim('dynamics');
q_ev = out.q_ev; % or if sim doesn't finish: q_ev = load('q_ev.mat');
%plot_robot(q_ev)

%% Plot Theta evolution
plot(ts,Theta0)
hold on
plot(q_ev.Time,q_ev.Data(:,1))
plot(ts,Theta1)
plot(q_ev.Time,q_ev.Data(:,2))

%% Save output
n_step = q_ev.time(end)*30;
stop_time = q_ev.time(end);
time_vect = linspace(0,stop_time,n_step);
q_ev_res = resample(q_ev,time_vect);
data = [time_vect', q_ev_res.Data(:,1), q_ev_res.Data(:,2)];
writematrix(data,'black_swing_sim_newMdef_k11_b05_BC35_off_n56_197_long.csv');
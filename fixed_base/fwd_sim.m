%% Init
clear
addpath('automatically_generated/fixed')

%%
% Object properties
global p_vals G_dir
p_vals = [0.6, 0.23, 0.61, 0.02];
G_dir = 0;

k_obj = 0.01;
beta_obj = 0.0505;

global K Theta_bar D
global G_Scale BC_Scale
G_Scale = 1.0;
BC_Scale = 1.6886;

H = [1, 1/2; 1/2, 1/3];
K = k_obj*H;
Theta_bar = [-0.5;3];
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
hold off

%% Plot steady state comparison
for i = 3:3:21 % 1 = -165deg, 23 = 165 deg. 6 = -90deg, 12 = 0deg, 18 = 90deg
    G_dir = -Gamma(i); % Note- use -ve Gamma since since data is robot angle
    out = sim('dynamics');
    q_ev = out.q_ev;
    plot_config(Theta0(i),Theta1(i), 0.5)
    hold on
    plot_config(q_ev.Data(end,1), q_ev.Data(end,2), 1)
    hold on
    scatter(X_mid(i),Z_mid(i),30,[0.4660 0.6740 0.1880],'+')
    scatter(X_end(i),Z_end(i),30,[0 0.4470 0.7410],'+')
end

%% Save output
n_step = q_ev.time(end)*30;
stop_time = q_ev.time(end);
time_vect = linspace(0,stop_time,n_step);
q_ev_res = resample(q_ev,time_vect);
data = [time_vect', q_ev_res.Data(:,1), q_ev_res.Data(:,2)];
writematrix(data,'black_swing_sim_k078_b0455_BC16536_off_n745_562.csv');
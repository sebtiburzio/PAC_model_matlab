%% Init
clear
addpath('automatically_generated/fixed')
global k_obj K p_vals Theta_bar
global beta_obj D BC_Scale

%%
% Load predefined object parameters
load('../object_parameters/black_weighted.mat')

%%
% % Manually defined object parameters (overwrites loaded parameters)
% % Static properties
% p_vals = [0.6, 0.23, 0.6, 0.02]';
% k_obj = 0.1965;
% Theta_bar = [0.1421; 1.2391];
% 
% % Dynamic properties
% beta_obj = 0.0547;
% BC_Scale = 1.8971;

%%
H = [1, 1/2; 1/2, 1/3];
K = k_obj*H;
D = beta_obj*H;

% Simulation set up
global G_Scale G_dir
G_Scale = 1.0;
G_dir = 0.0;
% Initial condition
x_0 = [1e-3; 1e-3];
dx_0 = [0; 0];

%% Simulate full dynamic system
out = sim('ss_solver');
q_ev = out.q_ev; % if sim doesn't finish: q_ev = load('q_ev.mat');
%plot_robot(q_ev)

%% Plot Theta evolution
% plot(ts,Theta0)
hold on
plot(q_ev.Time,q_ev.Data(:,1))
% plot(ts,Theta1)
plot(q_ev.Time,q_ev.Data(:,2))
hold off

%% Save dynamic evolution output
n_step = q_ev.time(end)*30;
stop_time = q_ev.time(end);
time_vect = linspace(0,stop_time,n_step);
q_ev_res = resample(q_ev,time_vect);
data = [time_vect', q_ev_res.Data(:,1), q_ev_res.Data(:,2)];
writematrix(data,'orange_weighted_swing_sim_B2.csv');

%% Plot steady state comparison
% [~, ss_range] = min(abs(Gamma-[3*pi/4 pi/2 pi/4 0 -pi/4 -pi/2-3*pi/4])); % Select indexes in Gamma closest to desired plot angles
[~, ss_range] = min(abs(Gamma-(6:11)*pi/12));
for i = ss_range
    G_dir = -Gamma(i); % Note- use -ve Gamma since since data is robot angle
    out = sim('ss_solver');
    q_ev = out.q_ev;
    plot_config(Theta0(i),Theta1(i), 0.5,  Gamma(i))
    hold on
    plot_config(q_ev.Data(end,1), q_ev.Data(end,2), 1,  Gamma(i))
    hold on
%     scatter(X_mid(i),Z_mid(i),30,[0.4660 0.6740 0.1880],'+')
%     scatter(X_end(i),Z_end(i),30,[0 0.4470 0.7410],'+')
end

%% Visualise steady state endpoint error
del_endpt = zeros(length(Gamma),1);
figure
hold on
for i = 1:length(Gamma)
    G_dir = -Gamma(i); % Note- use -ve Gamma since since data is robot angle
    out = sim('ss_solver');
    q_ev = out.q_ev;
    endpt_model = fk_fcn(p_vals, [q_ev.Data(end,1), q_ev.Data(end,2)]',1,0);
    del_endpt(i) = norm([X_end(i); Z_end(i)]-endpt_model);
    scatter(X_end(i), Z_end(i), 50,'kx')
    scatter(endpt_model(1), endpt_model(2), 20, [0 0.4470 0.7410], 'filled')
    plot([endpt_model(1) X_end(i)], [endpt_model(2) Z_end(i)],'k:')
end
axis equal
hold off 
figure
scatter(Gamma,del_endpt)
axis square
% Fit quadratic to error
coeffs = polyfit(Gamma,del_endpt,2);
hold on
xpts = linspace(-pi,pi);
plot(xpts,coeffs(1)+coeffs(2)*xpts+coeffs(3)*xpts.^2)
hold off

%% Determine endpoint location of model for whole Phi range
Phi_range = -135:-1;
Phi_range = [Phi_range 1:135];
model_endpt = [];
for i = Phi_range
    G_dir = i*pi/180; % Note- use -ve Gamma since since data is robot angle
    out = sim('ss_solver');
    q_ev = out.q_ev;
    model_endpt = [model_endpt fk_fcn(p_vals,[q_ev.Data(end,1), q_ev.Data(end,2)]',1,0)];
end

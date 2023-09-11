%% Init
clear
clear global
rmpath('../floating_base/automatically_generated')
addpath('automatically_generated')
global k_obj K p_vals Theta_bar
global beta_obj D

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
% x_0 = [Theta0(1); Theta1(1)];
% dx_0 = [dTheta0(1); dTheta1(1)];

%% Simulate full dynamic system
out = sim('dynamics');
q_ev = out.q_ev; % if sim doesn't finish: q_ev = load('q_ev.mat');
%plot_robot(q_ev)

%% Plot Theta evolution
% plot(ts,Theta0,'LineWidth',2)
hold on
plot(q_ev.Time,q_ev.Data(:,1),'LineWidth',2)
% plot(ts,Theta1,'LineWidth',2)
plot(q_ev.Time,q_ev.Data(:,2),'LineWidth',2)
hold off
xlabel('t (s)')
ylabel('\theta (rad)')
legend('\theta_0 (measured)','\theta_0 (simulated)', '\theta_1 (measured)','\theta_1 (simulated)')
ax = gca;
ax.FontSize = 24;

%% Make full size and check xlim before export
exportgraphics(ax,'D:\Study\Thesis\Report\images\dynamic_id\full_dyn_orange_weighted_theta_ev.eps')

%% Save dynamic evolution output
n_step = q_ev.time(end)*30;
stop_time = q_ev.time(end);
time_vect = linspace(0,stop_time,n_step);
q_ev_res = resample(q_ev,time_vect);
data = [time_vect', q_ev_res.Data(:,1), q_ev_res.Data(:,2)];
writematrix(data,'orange_short_unweighted_swing_sim.csv');

%% Plot steady state comparison - floating base frame
[~, ss_range] = min(abs(Gamma-[3*pi/4 pi/2 pi/4 0 -pi/4 -pi/2 -3*pi/4])); % Select indexes in Gamma closest to desired plot angles
% [~, ss_range] = min(abs(Gamma-(4:4)*pi/12));
for i = ss_range
    G_dir = -Gamma(i); % Note- use -ve Gamma since since data is robot angle
    out = sim('ss_solver');
    q_ev = out.q_ev;
    plot_config(Theta0(i),Theta1(i), 0.5,  Gamma(i))
    hold on
    plot_config(q_ev.Data(end,1), q_ev.Data(end,2), 1,  Gamma(i))
    hold on
    scatter(X_mid(i)*cos(Gamma(i))+Z_mid(i)*sin(Gamma(i)),-X_mid(i)*sin(Gamma(i))+Z_mid(i)*cos(Gamma(i)),150,[0.4660 0.6740 0.1880],'x','LineWidth',1)
    scatter(X_end(i)*cos(Gamma(i))+Z_end(i)*sin(Gamma(i)),-X_end(i)*sin(Gamma(i))+Z_end(i)*cos(Gamma(i)),150,[0 0.4470 0.7410],'x','LineWidth',1)
end
xlabel('x_B (m)')
ylabel('y_B (m)')
xticks(-0.6:0.1:0.6)
ax = gca;
ax.FontSize = 26;

%% 
exportgraphics(ax,'D:\Study\Thesis\Report\images\static_ID_assessment\orange_weighted_static_eqs.eps')

%% Plot steady state comparison - fixed base frame
[~, ss_range] = min(abs(Gamma-[3*pi/4 pi/2 pi/4 0 -pi/4 -pi/2 -3*pi/4])); % Select indexes in Gamma closest to desired plot angles
% [~, ss_range] = min(abs(Gamma-(6:11)*pi/12));
for i = ss_range
    G_dir = -Gamma(i); % Note- use -ve Gamma since since data is robot angle
    out = sim('ss_solver');
    q_ev = out.q_ev;
    plot_config(Theta0(i),Theta1(i), 0.5, 0)
    hold on
    plot_config(q_ev.Data(end,1), q_ev.Data(end,2), 1, 0)
    hold on
    scatter(X_mid(i),Z_mid(i),40,[0.4660 0.6740 0.1880],'x')
    scatter(X_end(i),Z_end(i),40,[0 0.4470 0.7410],'x')
end

%% Visualise steady state endpoint error
del_endpt = zeros(length(Gamma),1);
% figure
% hold on
for i = 1:length(Gamma)
    G_dir = -Gamma(i); % Note- use -ve Gamma since since data is robot angle
    out = sim('ss_solver');
    q_ev = out.q_ev;
    endpt_model = fk_fcn(p_vals, [q_ev.Data(end,1), q_ev.Data(end,2)]',1,0);
    del_endpt(i) = norm([X_end(i); Z_end(i)]-endpt_model);
%     scatter(X_end(i), Z_end(i), 50,'kx')
%     scatter(endpt_model(1), endpt_model(2), 20, [0 0.4470 0.7410], 'filled')
%     plot([endpt_model(1) X_end(i)], [endpt_model(2) Z_end(i)],'k:')
end
% axis equal
% hold off 
figure
scatter(Gamma,del_endpt,60,[0 0.4470 0.7410],'filled')
grid on
box on
xlim([-pi, pi])
xticks(-pi:pi/4:pi)
xticklabels({'-\pi','-3\pi/4','-\pi/2','-\pi/4','0','\pi/4','\pi/2','3\pi/4','\pi'})
xlabel('\phi (rad)')
ylabel('\Delta p_e (m)')
ax=gca;
ax.FontSize=26;

%% Plot linear fit
hold on
plot([0,3*pi/4],[0,0.0275*3*pi/4],'Color',[1.0 0.5 0],'LineWidth',2)
plot([0,-3*pi/4],[0,0.0275*3*pi/4],'Color',[1.0 0.5 0],'LineWidth',2)
hold off
% % Fit quadratic to error
% coeffs = polyfit(Gamma,del_endpt,2);
% hold on
% xpts = linspace(-pi,pi);
% plot(xpts,coeffs(1)+coeffs(2)*xpts+coeffs(3)*xpts.^2)
% hold off

%% Determine endpoint location of model for whole Phi range
% Phi_range = -135:-1;
% Phi_range = [Phi_range 1:135];
Phi_range = 0:1:359;
theta_eq = [];
midpt_eq = [];
endpt_eq = [];
theta_eq_prev = [1e-3; 1e-3];
for i = Phi_range
    G_dir = i*pi/180; % Note- use -ve Gamma since since data is robot angle
    x_0 = theta_eq_prev;
    out = sim('ss_solver');
    q_ev = out.q_ev;
    theta_eq_prev = [q_ev.Data(end,1), q_ev.Data(end,2)]';
    theta_eq = [theta_eq theta_eq_prev];
    midpt_eq = [midpt_eq fk_fcn(p_vals,theta_eq_prev,0.5,0)];
    endpt_eq = [endpt_eq fk_fcn(p_vals,theta_eq_prev,1,0)];
end

%%
save('equilibria_positive_dir','Phi_range','theta_eq','midpt_eq','endpt_eq')

%%
load('equilibria_negative_dir.mat');
plot(theta_eq(1,:));
%%
max_neg = 210;
Phi_range_n = flip(Phi_range(1:max_neg));
theta_eq_n = flip(theta_eq(:,1:max_neg),2);
midpt_eq_n = flip(midpt_eq(:,1:max_neg),2);
endpt_eq_n = flip(endpt_eq(:,1:max_neg),2);
%%
load('equilibria_positive_dir.mat');
plot(theta_eq(1,:));
%%
max_pos = 225;
Phi_range = [Phi_range_n Phi_range(2:max_pos)];
theta_eq = [theta_eq_n theta_eq(:,2:max_pos)];
endpt_eq = [endpt_eq_n endpt_eq(:,2:max_pos)];
midpt_eq = [midpt_eq_n midpt_eq(:,2:max_pos)];

%%
save('equilibria_span','Phi_range','theta_eq','midpt_eq','endpt_eq')
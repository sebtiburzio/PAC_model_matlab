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
% load('../object_parameters/full_dynamic_id/orange_weighted_dyn.mat')

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

%% For comparing to imported measured dynamic evolution
% Import the full theta state data csv as column vectors first
x_0 = [Theta0(1);Theta1(1)];
dx_0 = [dTheta0(1);dTheta1(1)];
mdl = 'dynamics';
open_system(mdl)
set_param(mdl,"StopTime",string(ts(end)))

%% Simulate full dynamic system
out = sim('dynamics');
q_ev = out.q_ev; % if sim doesn't finish: q_ev = load('q_ev.mat');
%plot_robot(q_ev)

%% Plot/compare evolutions
plot(ts,[Theta0 Theta1])
hold on
plot(q_ev.Time,[q_ev.Data(:,1) q_ev.Data(:,2)])
hold off
xlabel('t (s)','Interpreter','latex')
ylabel('$\Theta$ (rad)','Interpreter','latex')
xlim([0 ts(end)])
ylim([-3 5])
legend('$\theta_0$ (measured)','$\theta_1$ (measured)','$\theta_0$ (simulated)','$\theta_1$ (simulated)','Interpreter','latex')
ax = gca;
set(ax, 'FontSize', 32)
set(ax, 'TickLabelInterpreter', 'latex')
lines = findobj(gcf, 'Type', 'line');
for i = 1:length(lines)
    set(lines(i), 'markersize', 10, 'linewidth', 2);
end
grid on
box on
% set(gcf, 'Position', [303 495 1115 331])

%% Save dynamic evolution output
n_step = q_ev.time(end)*30;
stop_time = q_ev.time(end);
time_vect = linspace(0,stop_time,n_step);
q_ev_res = resample(q_ev,time_vect);
data = [time_vect', q_ev_res.Data(:,1), q_ev_res.Data(:,2)];
writematrix(data,'black_weighted_swing_sim.csv');

%% Plot steady state comparison - floating base frame
% Import the steady state measurements data csv as column vectors first
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
xlabel('$x_B$ (m)','Interpreter','latex')
ylabel('$y_B$ (m)','Interpreter','latex')
xticks(-0.6:0.1:0.6)
yticks(-1:0.1:1)
ax = gca;
set(ax, 'FontSize', 30)
set(ax, 'TickLabelInterpreter', 'latex')

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
%% Plot of steady state endpoint error magnitude
figure
scatter(-Gamma,del_endpt,60,[0 0.4470 0.7410],'filled') % For the report, plot -ve Gamma on x-axis because rotation is opposite
xlim([-pi, pi])
xticks(-pi:pi/4:pi)
xticklabels({'$-\pi$','$\frac{-3\pi}{4}$','$\frac{-\pi}{2}$','$\frac{-\pi}{4}$','0','$\frac{\pi}{4}$','$\frac{\pi}{2}$','$\frac{3\pi}{4}$','$\pi$'})
xlabel('$\phi$ (rad)','Interpreter','latex')
ylabel('$\Delta p_e$ (m)','Interpreter','latex')
grid on
box on
ax = gca;
set(ax, 'FontSize', 38)
set(ax, 'TickLabelInterpreter', 'latex')

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

%% Export endpoint location of model for whole Phi range
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

%% Affine Curvature Visualisation for defense presentation
% This has nothing to do with the dynamic model, just visualising AC kinematics
theta_start = [5*pi/4 -pi];
theta_end = [5*pi/4 -2.8*pi];

fig = figure;
fig.Position = [100 100 1200 360];

frames = 30;

for i = 0:frames
    theta_0 = -(-1e-3+theta_start(1)+i*(theta_end(1)-theta_start(1))/frames);
    theta_1 = -(-1e-3+theta_start(2)+i*(theta_end(2)-theta_start(2))/frames);
    subplot(1,2,1)
    plot_config(-theta_0,-theta_1,1,0)
    subplot(1,2,2)
    plot([0 1],[theta_0 theta_0+theta_1],Color=[1.0 0.5 0.0],LineWidth=2)
    hold on
    plot([0 1],[theta_0 theta_0],Color=[0 0.4470 0.7410],LineWidth=2)
    plot([0 1],[0 theta_1],Color=[0.9290 0.6940 0.1250],LineWidth=2)
    hold off
    xlim([-0.1,1.1])
    ylim([-10,10])
    ylabel('\theta (rad)')
    xlabel('s')
    legend(sprintf('c   = %.2f',theta_0+theta_1),sprintf('\\theta_0 = %.2f',theta_0),sprintf('\\theta_1 = %.2f',theta_1),Location="southeast")
    grid on
    pos = get(gca, 'Position');
    pos(1) = 0.53;
    set(gca, 'Position', pos)
    drawnow
    exportgraphics(fig,'./frames/' + string(i) + '.png','Resolution',300)
end
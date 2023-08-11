%%
clear
addpath('automatically_generated')
global k_obj K p_vals Theta_bar

%%
% SETUP

%% Object properties
% Load predefined object parameters
load('../object_parameters/black_weighted.mat')

% % Manually defined object parameters (overwrites loaded parameters)
% p_vals = [0.6, 0.23, 0.6, 0.02]';
% k_obj = 0.1799;
% Theta_bar = [-0.0463, 1.3731];

Pi = [k_obj; Theta_bar];
H = [1, 1/2; 1/2, 1/3];
K = k_obj*H;

%% Optimisation set up
global goal
% options = optimoptions('fmincon','EnableFeasibilityMode',true,'SubproblemAlgorithm','cg');

% Constraints
lb = [-Inf,-Inf, -1.2, 0.333, -3*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1.2, 1.2, 3*pi/4];
global radial_constraint
radial_constraint = 0.55; % Centered on Joint1

%%
% SOLVERS

%%
% Position only
q_0 = [1e-1; 1e-1; -0.1; 0.5; -2*pi/4];
goal = [0; 0];
[q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon)
goals = goal;
results = exitflag;
path = [q_st(3); q_st(4); q_st(5)];
curv = [q_st(1); q_st(2)];

%%
% Position and orientation
q_0 = [1e-1; 1e-1; 0.0; 0.9; 0];
goal = [0.14; 0.333; 0];
[q_st,fval,exitflag] = fmincon(@fa,q_0,[],[],[],[],lb,ub,@nonlcon)
goals = goal;
results = exitflag;
path = [q_st(3); q_st(4); q_st(5)];
curv = [q_st(1); q_st(2)];

%%
%Line
q_0 = [1e-3; 1e-3; 0.0; 0.0; 0.0];
goals = [];
curv = [];
path = [];
results = [];
for i = 0:9
    % goal = [0.2+0.07*i; 0.333];
    goal = [0.4; 0.1+0.07*i];
    goals = [goal, goals];
    [q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon);
    results = [results exitflag];
    path = [path, [q_st(3); q_st(4); q_st(5)]];
    curv = [curv, [q_st(1); q_st(2)]];
    q_0 = q_st;
    plot_config(q_st,i/10+0.1)
    hold on
end
hold off

%%
%Circle
q_0 = [1e-3; 1e-3; 0.0; 0.0; 0.0];
curv = [];
path = [];
for i = 0:pi/6:2*pi
    goal = [0.4+0.2*cos(i); 0.333-0.2*sin(i)];
    q_st = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon)
    q_0 = q_st;
    path = [path, [q_st(3); q_st(4); q_st(5)]];
    curv = [curv, [q_st(1); q_st(2)]];
    plot_config(q_st,i/6.5+0.03)
    hold on
end
hold off

%%
% Grid - solve and plot
goals = [];
curv = [];
path = [];
results = [];
lb = [-Inf,-Inf, -1.2, 0, -3*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1.2, 1.2, 3*pi/4];
radial_constraint = 0.55; % Centered on Joint1
xg_start = -0.1;
xg_spacing = 0.02;
xg_end = 0.1;
zg_start = 0.55;
zg_spacing = 0.1;
zg_end = 0.55;
% Use vertical steady state from model simulation as q_0 at first point
% q_0_row = [-0.357303109718888; 1.373131904009318; xg_start+0.029802598965446; zg_start+0.595768690034612; 0];
q_0 = [1e-3;1e-3;0;0.8;0];
for zg = zg_start:zg_spacing:zg_end
    for xg = xg_start:xg_spacing:xg_end
        goal = [xg; zg];
        goals = [goals, goal];

%         % If at start of row, use previous row start solution as q_0
%         if (xg == xg_start)
%             q_0 = q_0_row;
%         % Otherwise use solution from previous point
%         else
%             q_0 = q_st;
%         end

        [q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon)

%         % If at start of row, save solution to be q_0 at start of next row
%         if (xg == xg_start)
%             q_0_row = q_st;
%         end

        results = [results, exitflag];
        path = [path, [q_st(3); q_st(4); q_st(5)]];
        curv = [curv, [q_st(1); q_st(2)]];
        scatter(goal(1),goal(2),50,'kx')
        hold on
        plot_config(q_st,1);%zg/0.75+0.05)
        hold on
        endpt = fk_fcn(p_vals, q_st, 1, 0);
        plot([endpt(1) goal(1)], [endpt(2) goal(2)],'k:')
    end
end
plot(goals(1,:),goals(2,:),Color=[1.0 0.75 0.75])
hold off

% Plot constraints
hold on
xline([lb(3) ub(3)],'r')
yline([lb(4) ub(4)],'r')
th = 0:pi/50:2*pi;
xunit = radial_constraint * cos(th);
yunit = radial_constraint * sin(th) + 0.333;
h = plot(xunit, yunit,'r');
hold off

%% 
% PLOTTING

%%
% Plot from saved single result
marker = [-0.278, 0.288];
q_res = [curv(:,1); path(:,1)];
scatter(goals(1,1),goals(2,1),50,'kx')
hold on
plot_config(q_res,1)
hold on
endpt = fk_fcn(p_vals, q_res, 1, 0);
plot([endpt(1) goals(1,1)], [endpt(2) goals(2,1)],'k:')
scatter(marker(1),marker(2),10,[0.3010 0.7450 0.9330],'filled')
hold off

%%
% Plot from saved path
endpts = [];
for i = 1:length(results)
    q_i = [curv(:,i); path(:,i)];
    scatter(goals(1,i),goals(2,i),50,'kx')
    hold on
    plot_config(q_i,goals(2,i)/goals(2,end))
    hold on
    endpt = fk_fcn(p_vals, q_i, 1, 0);
    endpts = [endpts endpt];
    plot([endpt(1) goals(1,i)], [endpt(2) goals(2,i)],'k:')
end
plot(goals(1,:),goals(2,:),Color=[0.75 0.75 1.0])
% plot(end_pos_x,end_pos_z,Color=[0.3010 0.7450 0.9330])
hold off

%%
% Plot model and real endpt paths
figure
plot(endpts(1,:),endpts(2,:))
hold on
scatter(endpts(1,:), endpts(2,:), 10, [0 0.4470 0.7410], 'filled')
plot(end_pos_x, end_pos_z, Color=[0.3010 0.7450 0.9330])
axis equal
xlim([-0.75,0])
hold off

%%
% Plot joined LHS and RHS model and real endpt paths
clear
load('0724\black_cable\k000100To000000Ti000000\black_grid_horiz_LHS.mat')
endpts = [];
for i = 1:length(results)
    q_i = [curv(:,i); path(:,i)];
    endpt = fk_fcn(p_vals, q_i, 1, 0);
    endpts = [endpts endpt];
end
endpts = (flip(endpts'))';
load('0724\black_cable\k000100To000000Ti000000\black_grid_horiz_RHS.mat')
for i = 1:length(results)
    q_i = [curv(:,i); path(:,i)];
    endpt = fk_fcn(p_vals, q_i, 1, 0);
    endpts = [endpts endpt];
end
LHS_meas = readmatrix("0724\black_cable\k000100To000000Ti000000\black_grid_horiz_LHS_measurements.csv");
RHS_meas = readmatrix("0724\black_cable\k000100To000000Ti000000\black_grid_horiz_RHS_measurements.csv");
figure
plot(endpts(1,:),endpts(2,:))
hold on
scatter(endpts(1,:),endpts(2,:), 10, [0 0.4470 0.7410], 'filled')
plot([flip(LHS_meas(:,6)); RHS_meas(:,6)],[flip(LHS_meas(:,7)); RHS_meas(:,7)],Color=[0.3010 0.7450 0.9330])
axis equal
xlim([-0.75,0.75])
hold off

%%
% OUTPUT

writematrix(path','Xn494Yn200Z662P145.csv');
save('Xn043Yn200Z847P154','p_vals','Pi', 'goals', 'results', 'path', 'curv', 'lb', 'ub', 'radial_constraint');

%%
% Cost function characterisation
global theta0_grid
theta0_grid = -9.5:1:9.5;
global theta1_grid
theta1_grid = -9.5:1:9.5;
global x_grid
x_grid = -0.3:0.1:0.3;
global z_grid
z_grid = 0.7:0.1:(0.35+0.6);
global phi_grid
phi_grid = -3*pi/4:pi/12:3*pi/4;

goal_calc = [0;0.35];
global cost_grid
cost_grid = Inf*ones(length(theta0_grid),length(theta1_grid),length(x_grid),length(z_grid),length(phi_grid));

theta0_idx = 1;
for theta0_calc = theta0_grid
    theta1_idx = 1;
    for theta1_calc = theta1_grid
        x_idx = 1;
        for x_calc = x_grid
            z_idx = 1;
            for z_calc = z_grid
                phi_idx = 1;
                for phi_calc = phi_grid
                    q_calc = [theta0_calc;theta1_calc;x_calc;z_calc;phi_calc];
                    G_eval = G_fcn(p_vals,q_calc);
                    if all(abs((G_eval(1:2) + K*[theta0_calc-Theta_bar(1); theta1_calc-Theta_bar(2)]))) < 1e-6 % 1e-6 is default fmincon constraint tolerance
                        cost_grid(theta0_idx,theta1_idx,x_idx,z_idx,phi_idx) = norm(fk_fcn(p_vals, q_calc, 1, 0) - goal_calc);
                    end
                    phi_idx = phi_idx + 1;
                end
                z_idx = z_idx + 1;
            end
            x_idx = x_idx + 1;
        end
        theta1_idx = theta1_idx + 1;
    end
    theta0_idx = theta0_idx + 1;
end

%% Save cost grid
save('cost_grid_test', 'p_vals', 'k_obj', 'Theta_bar', 'goal_calc', ...
    'theta0_grid', 'theta1_grid', 'x_grid', 'z_grid', 'phi_grid', 'cost_grid');

%%
sample_cg(-0.5,-0.5,0,0,pi/2)

%%
function cost = sample_cg(at_th0,at_th1,at_x,at_z,at_phi)
    global theta0_grid
    global theta1_grid
    global x_grid
    global z_grid
    global phi_grid
    global cost_grid

    [~,th0_idx] = min(abs(theta0_grid-(at_th0)));
    [~,th1_idx] = min(abs(theta1_grid-(at_th1)));
    [~,x_idx] = min(abs(x_grid-(at_x)));
    [~,z_idx] = min(abs(z_grid-(at_z)));
    [~,phi_idx] = min(abs(phi_grid-(at_phi)));
    cost = cost_grid(th0_idx,th1_idx,x_idx,z_idx,phi_idx);
end

%%
% OPTIMISATION FUNCTION DEFINITIONS

% Objective function - minimise endpoint goal distance
function obj = f(q)
    global p_vals goal
    Phi_cost = 0;%0.02*abs(q(5));
    endpt_cost = norm(fk_fcn(p_vals, q, 1, 0) - goal);
%     if endpt_cost < 0.01
%         endpt_cost = 0;
%     end
    obj = endpt_cost + Phi_cost;
end

% Objective function - minimise endpoint goal distance inc orientation TODO - correct measure of orientation error
function obj = fa(q)
    global p_vals goal
    obj = norm(fka_fcn(p_vals, q, 1, 0) - goal);
end

% Constraints
function [c,ceq] = nonlcon(q)
    % Steady state of object
    global p_vals K Theta_bar radial_constraint
    G_eval = G_fcn(p_vals,q);
    ceq = G_eval(1:2) + K*[q(1)-Theta_bar(1); q(2)-Theta_bar(2)];
    % Circular workspace region
    c = q(3)^2 + (q(4)-0.333)^2 - radial_constraint^2;
end


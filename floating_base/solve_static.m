%%
clear
addpath('automatically_generated')

%%
% SETUP

%Object properties
global p_vals k Theta_bar
p_vals = [0.6, 0.23, 0.6, 0.02]';
k = 0.1;
Theta_bar = [-0.5, 3];
Pi = [k Theta_bar];

global goal
% Constraints
lb = [-Inf,-Inf, -1, 0.333, -2*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1, 0.85, 2*pi/4];
global radial_constraint
radial_constraint = 0.85;

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
q_0 = [1e-3; 1e-3; 0.0; 0.9; 0];
goals = [];
curv = [];
path = [];
results = [];
lb = [-Inf,-Inf, -1, 0.33, -2*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1, 1.2, 2*pi/4];
for zg = 0.1:0.2:0.5
    for xg = 0.0:0.22:0.66
        goal = [xg; zg];
        goals = [goals, goal];
        [q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon);
        results = [results, exitflag];
        %q_0 = q_st;
        path = [path, [q_st(3); q_st(4); q_st(5)]];
        curv = [curv, [q_st(1); q_st(2)]];
        scatter(goal(1),goal(2),50,'kx')
        hold on
        plot_config(q_st,zg/0.4+0.1)
        hold on
        endpt = fk_fcn(p_vals, q_st, 1, 0);
        plot([endpt(1) goal(1)], [endpt(2) goal(2)],'k:')
    end
end
plot(goals(1,:),goals(2,:),Color=[1.0 0.75 0.75])
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
% OUTPUT

writematrix(path','Xn494Yn200Z662P145.csv');
save('Xn043Yn200Z847P154','p_vals','Pi', 'goals', 'results', 'path', 'curv', 'lb', 'ub', 'radial_constraint');

%%
% OPTIMISATION DEFINITIONS

% Objective function - minimise endpoint goal distance
function obj = f(q)
    global p_vals goal
    obj = norm(fk_fcn(p_vals, q, 1, 0) - goal);
end

% Objective function - minimise endpoint goal distance inc orientation TODO - correct measure of orientation error
function obj = fa(q)
    global p_vals goal
    obj = norm(fka_fcn(p_vals, q, 1, 0) - goal);
end

% Constraints
function [c,ceq] = nonlcon(q)
    % Steady state of object
    global p_vals k Theta_bar radial_constraint
    G_eval = G_fcn(p_vals,q);
    ceq = G_eval(1:2) + k.*[1, 1/2; 1/2, 1/3]*[q(1)-Theta_bar(1); q(2)-Theta_bar(2)];
    % Circular workspace region
    c = q(3)^2 + (q(4)-0.333)^2 - radial_constraint^2;
end

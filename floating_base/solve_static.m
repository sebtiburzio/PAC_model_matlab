%%
clear
addpath('automatically_generated')

%%
global p_vals k Theta_bar
p_vals = [0.6, 0.23, 0.6, 0.02]';
k = 0.01;
Theta_bar = [-0.5,3];
Pi = [k Theta_bar];

global goal
q_0 = [1e-3; 1e-3; 0.0; 0.9; pi/4];

lb = [-Inf,-Inf, -1, 0.333, 0*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1, 1.0, 4*pi/4];
global radial_constraint
radial_constraint = 0.55;

%%
% Position only
goal = [-0.153; 0.333];
[q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon)
goals = goal;
results = exitflag;
path = [q_st(3); q_st(4); q_st(5)];
curv = [q_st(1); q_st(2)];

%%
% Position and orientation
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
% General grid
q_0 = [1e-3; 1e-3; 0.0; 0.65; 0.0];
goals = [];
curv = [];
path = [];
results = [];
lb = [-Inf,-Inf, -1, 0.33, -2*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1, 1.2, 2*pi/4];
for zg = 0.1:0.2:0.5
    for xg = 0:0.22:0.66
        goal = [xg; zg];
        goals = [goals, goal];
        [q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon);
        results = [results, exitflag];
        q_0 = q_st;
        path = [path, [q_st(3); q_st(4); q_st(5)]];
        curv = [curv, [q_st(1); q_st(2)]];
        scatter(goal(1),goal(2),50,'kx')
        hold on
        plot_config(q_st,zg/0.75+0.05)
        hold on
        endpt = fk_fcn(p_vals, q_st, 1, 0);
        plot([endpt(1) goal(1)], [endpt(2) goal(2)],'k:')
    end
end
plot(goals(1,:),goals(2,:),Color=[1.0 0.75 0.75])
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
% Path output
writematrix(path','black_grid.csv');
save('black_grid','p_vals','Pi', 'goals', 'results', 'path', 'curv', 'lb', 'ub', 'radial_constraint');

%%
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
    ceq = G_eval(1:2) + k.*[q(1)-Theta_bar(1); q(2)-Theta_bar(2)];
    % Circular workspace region
    c = q(3)^2 + (q(4)-0.333)^2 - radial_constraint^2;
end

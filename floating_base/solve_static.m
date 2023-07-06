%%
clear
addpath('automatically_generated')

%%
global p_vals k Theta_bar
p_vals = [0.6, 0.23, 0.61, 0.02]';
k = 0.01;
Theta_bar = [-0.5,3];

global goal
q_0 = [1e-3; 1e-3; 0.0; 0.0; 0.0];

lb = [-Inf,-Inf, -0.4, 0.333, -3*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 0.855, 1.19, pi/4];

%%
% Position only
goal = [0.5; 0.3];
q_st = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon)

%%
% Position and orientation
goal = [0.8; 0.3; pi/2];
q_st = fmincon(@fa,q_0,[],[],[],[],lb,ub,@nonlcon)

%%
%Line
q_0 = [1e-3; 1e-3; 0.0; 0.0; 0.0];
for i = 0:9
    goal = [0.1+0.09*i; 0.1];
%     goal = [0.4; 0.1+0.07*i];
    q_st = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon)
    q_0 = q_st;
    plot_config(q_st,i/10+0.1)
    hold on
end
hold off

%%
%Circle
q_0 = [1e-3; 1e-3; 0.0; 0.0; 0.0];
for i = 0:pi/6:2*pi
    goal = [0.5+0.1*cos(i); 0.6-0.1*sin(i)];
    q_st = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon)
    q_0 = q_st;
    plot_config(q_st,i/6.5+0.03)
    hold on
end
hold off

%%
% Plot constraints
hold on
xline([lb(3) ub(3)],'r')
yline([lb(4) lb(4)],'r')
th = 0:pi/50:2*pi;
xunit = 0.85 * cos(th);
yunit = 0.85 * sin(th) + 0.333;
h = plot(xunit, yunit,'r');
hold off

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
    global p_vals k Theta_bar
    G_eval = G_fcn(p_vals,q);
    ceq = G_eval(1:2) + k.*[q(1)-Theta_bar(1); q(2)-Theta_bar(2)];
    % Circular workspace region
    c = q(3)^2 + (q(4)-0.333)^2 - 0.85^2;
end







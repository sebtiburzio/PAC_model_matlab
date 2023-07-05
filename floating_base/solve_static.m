%%
clear
addpath('automatically_generated')

%%
global p_vals k Theta_bar
p_vals = [0.6, 0.23, 0.61, 0.02]';
k = 0.01;
Theta_bar = [-0.5,3];

global goal
q_0 = [1e-3; 1e-3; 0; 0; 0];

lb = [-Inf,-Inf, -1, 0, -3*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1, 1, 3*pi/4];

%%
% Position only
goal = [0.5; 0.5];
q_st = fmincon(@f,q_0,[],[],[],[],lb,ub,@static_model)

%%
% Position and orientation
goal = [0.5; 0.5; pi/2];
q_st = fmincon(@fa,q_0,[],[],[],[],lb,ub,@static_model)

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

% Constraint - steady state of object
function [c,ceq] = static_model(q)
    global p_vals k Theta_bar
    G_eval = G_fcn(p_vals,q);
    ceq = G_eval(1:2) + k.*[q(1)-Theta_bar(1); q(2)-Theta_bar(2)];
    c = [];
end







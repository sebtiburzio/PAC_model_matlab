%%
clear
clear global
rmpath('../fixed_base/automatically_generated')
addpath('automatically_generated')
global k_obj K p_vals Theta_bar

%%
% SETUP

%% Object properties
% Load predefined object parameters
load('../object_parameters/black_short_loop_100g.mat')

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
lb = [-Inf,-Inf, -1.2, 0.333, -2*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1.2, 1.2, 2*pi/4];
global radial_constraint
radial_constraint = 0.5; % Centered on Joint1

%%
% SOLVERS

%%
% Position only
q_0 = [1e-3; 1e-3; 0; 0; 0];
goal = [-0.3; 0.3];
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
    goal = [0.3-0.6/9*i; 0.3];
    goals = [goal, goals];
    [q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon);
    results = [results exitflag];
    path = [path, [q_st(3); q_st(4); q_st(5)]];
    curv = [curv, [q_st(1); q_st(2)]];
    q_0 = q_st;
    plot_config(q_st,1)%i/10+0.1)
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
endpts = [];
lb = [-Inf,-Inf, -1.2, 0, -3*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 1.2, 1.2, 3*pi/4];
radial_constraint = 0.5; % Centered on Joint1
xg_start = -0.7;
xg_spacing = 0.1;
xg_end = 0.7;
zg_start = 0.15;
zg_spacing = 0.1;
zg_end = 0.15;
for zg = zg_start:zg_spacing:zg_end
    figure;
    for xg = xg_start:xg_spacing:xg_end
        goal = [xg; zg];
        goals = [goals, goal];
       
        % Run optimisation with default q_0
        q_0 = [1e-3;1e-3;0.0;0.8;0];
        [q_st,fval,exitflag] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon);
        % Run optimisation with random q_0s to try to improve
        for i = 0:10
            q_0 = [1e-3;1e-3;xg-0.15+i*(0.3/10);zg+p_vals(3);0];
            [q_st_n,fval_n,exitflag_n] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon);
            if fval_n < fval
                q_st = q_st_n;
                fval = fval_n;
                exitflag = exitflag_n;
            end
        end

        results = [results, exitflag];
        path = [path, [q_st(3); q_st(4); q_st(5)]];
        curv = [curv, [q_st(1); q_st(2)]];
        scatter(goal(1),goal(2),50,'kx')
        hold on
        plot_config(q_st,1);%zg/0.75+0.05)
        hold on
        endpt = fk_fcn(p_vals, q_st, 1, 0);
        endpts = [endpts, endpt];
        plot([endpt(1) goal(1)], [endpt(2) goal(2)],'k:')
    end
    % Plot constraints
    hold on
    xline([lb(3) ub(3)],'r')
    yline([lb(4) ub(4)],'r')
    th = 0:pi/50:2*pi;
    xunit = radial_constraint * cos(th);
    yunit = radial_constraint * sin(th) + 0.333;
    plot(xunit, yunit,'r');
    saveas(gcf,string(zg*100))
end
hold off

%% Export the problem configuration and solutions
writematrix([path', curv', goals', endpts'],'sequence.csv');
save('vars','p_vals','Pi', 'goals', 'results', 'path', 'curv', 'endpts', 'lb', 'ub', 'radial_constraint');

%%
% Full orientation
goals = [];
curv = [];
path = [];
results = [];
endpts=[];
lb = [-Inf,-Inf, -2.0, 0, -4*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 2.0, 2.0, 3*pi/4];
radial_constraint = 2.0; % Centered on Joint1
for phig = -pi/2:pi/12:3*pi/4
    goal = [0.0; 0.45; phig];
    goals = [goals, goal];
   
    % Run optimisation with default q_0
    q_0 = [1e-3;1e-3;0.0;0.8;0];
    [q_st,fval,exitflag] = fmincon(@fa,q_0,[],[],[],[],lb,ub,@nonlcon);
%     % Run optimisation with random q_0s to try to improve
%     for i = 0:10
%         q_0 = [1e-3;1e-3;xg-0.15+i*(0.3/10);zg+p_vals(3);0];
%         [q_st_n,fval_n,exitflag_n] = fmincon(@f,q_0,[],[],[],[],lb,ub,@nonlcon);
%         if fval_n < fval
%             q_st = q_st_n;
%             fval = fval_n;
%             exitflag = exitflag_n;
%         end
%     end

    results = [results, exitflag];
    path = [path, [q_st(3); q_st(4); q_st(5)]];
    curv = [curv, [q_st(1); q_st(2)]];
    endpt = fk_fcn(p_vals, q_st, 1, 0);
    endpts = [endpts, endpt];
    plot_config(q_st,1);%zg/0.75+0.05)
    hold on
end

% Plot constraints
hold on
xline([lb(3) ub(3)],'r')
yline([lb(4) ub(4)],'r')
th = 0:pi/50:2*pi;
xunit = radial_constraint * cos(th);
yunit = radial_constraint * sin(th) + 0.333;
plot(xunit, yunit,'r');
hold off

%% Export the problem configuration and solutions
writematrix([path', curv', goals', endpts'],'sequence.csv');
save('full_range_centered','p_vals','Pi', 'goals', 'results', 'path', 'curv', 'lb', 'ub', 'radial_constraint');

%%
% Position and orientation for feedback testing
goals = [];
curv = [];
path = [];
results = [];
endpts = [];
lb = [-Inf,-Inf, -2.0, 0, -3*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 2.0, 2.0, 3*pi/4];
radial_constraint = 0.5; % Centered on Joint1
for xg = [-0.2, 0, 0.2]
    goal = [xg; 0.15; 0];
    goals = [goals, goal];
   
    % Run optimisation with default q_0
    q_0 = [1e-3;1e-3;0.0;0.8;0];
    [q_st,fval,exitflag] = fmincon(@fa,q_0,[],[],[],[],lb,ub,@nonlcon);
    % % Run optimisation with random q_0s to try to improve
    % for i = 0:10
    %     q_0 = [1e-3;1e-3;xg-0.15+i*(0.3/10);0.8;0];
    %     [q_st_n,fval_n,exitflag_n] = fmincon(@fa,q_0,[],[],[],[],lb,ub,@nonlcon);
    %     if fval_n < fval
    %         q_st = q_st_n;
    %         fval = fval_n;
    %         exitflag = exitflag_n;
    %     end
    % end

    results = [results, exitflag];
    path = [path, [q_st(3); q_st(4); q_st(5)]];
    curv = [curv, [q_st(1); q_st(2)]];
    endpt = fka_fcn(p_vals, q_st, 1, 0);
    endpts = [endpts, endpt];
    plot_config(q_st,1);%zg/0.75+0.05)
    hold on
end

% Plot constraints
hold on
xline([lb(3) ub(3)],'r')
yline([lb(4) ub(4)],'r')
th = 0:pi/50:2*pi;
xunit = radial_constraint * cos(th);
yunit = radial_constraint * sin(th) + 0.333;
plot(xunit, yunit,'r');
hold off

%%
% Position and orientation for feedforward demo
goals = [];
curv = [];
path = [];
results = [];
endpts = [];
lb = [-Inf,-Inf, -2.0, 0, -3*pi/4]; % Theta0, Theta1, X, Z, Phi
ub = [Inf, Inf, 2.0, 2.0, 3*pi/4];
radial_constraint = 0.65; % Centered on Joint1
offset_x = 0.8;
offset_z = 0.475;
% Upright hook
% goal_set = [0.025, 0.10, pi/2, 5;   % last element is number of points to interpolate to next goal
%             0.025, 0.15, pi/2, 15;
%             0.025, 0.15, 0, 5;
%             -0.080, 0.15, 0, 15;
%             -0.080, 0.15, -pi/2, 0]';
% RHS hook
goal_set = [-0.2, 0.13, 0, 5;
            -0.08, 0.13, 0, 5;
            -0.08, 0.025, 0, 5;
            -0.15, 0.025, 0, 12;
            -0.15, 0.025, -pi/4, 5]';
            % -0.15, -0.015, -pi/4, 0]';
% LHS hook
% goal_set = [0.1, 0.025, 0, 5;
%             0.15, 0.025, 0, 15;
%             0.15, 0.025, pi/2, 5;
%             0.15, -0.08, pi/2, 15]';
% Offset
goal_set = goal_set+[offset_x, offset_z, 0 ,0]';

goals = []
for i = 1:length(goal_set)-1
    steps = goal_set(4,i);
    goals = [goals [linspace(goal_set(1,i),goal_set(1,i+1),steps);linspace(goal_set(2,i),goal_set(2,i+1),steps);linspace(goal_set(3,i),goal_set(3,i+1),steps)] ];
end

q_0 = [1e-3;1e-3;0.0;0.8;pi/2];
for goal = goals
   
    [q_st,fval,exitflag] = fmincon(@fa,q_0,[],[],[],[],lb,ub,@nonlcon);
    q_0 = q_st;

    results = [results, exitflag];
    path = [path, [q_st(3); q_st(4); q_st(5)]];
    curv = [curv, [q_st(1); q_st(2)]];
    endpt = fka_fcn(p_vals, q_st, 1, 0);
    endpts = [endpts, endpt];
    % plot_config(q_st,0.19+length(results)*0.03);
    % Swap to below to only plot certain range of solns
    if length(results) >= 15 & length(results) <= 27
        plot_config(q_st,1);
    end
    hold on
end

% Plot hook
% Upright
% plot(offset_x+[0, -0.08, -0.08, 0.025, 0.025], offset_z+[0, 0, 0.15, 0.15, 0.11],LineWidth=2,Color=[0.5,0.5,0.5])
% RHS
plot(offset_x-[0, 0, 0.15, 0.15, 0.11], offset_z+[0, -0.08, -0.08, 0.025, 0.025],LineWidth=2,Color=[0.5,0.5,0.5])
% LHS
% plot(offset_x+[0, 0, 0.15, 0.15, 0.11], offset_z+[0, -0.08, -0.08, 0.025, 0.025],LineWidth=2,Color=[0.5,0.5,0.5])


% Plot constraints
hold on
xline([lb(3) ub(3)],'r')
yline([lb(4) ub(4)],'r')
th = 0:pi/50:2*pi;
xunit = radial_constraint * cos(th);
yunit = radial_constraint * sin(th) + 0.333;
plot(xunit, yunit,'r');
hold off

xlim([0.25,0.85])
xticks([0.3,0.4,0.5,0.6,0.7,0.8])
ylim([0.35,0.95])
xlabel('$x_B$ (m)','Interpreter','latex')
ylabel('$y_B$ (m)','Interpreter','latex')
ax = gca;
set(ax, 'FontSize', 30)
set(ax, 'TickLabelInterpreter', 'latex')
lines = findobj(gcf, 'Type', 'line');
for i = 1:length(lines)
    set(lines(i), 'markersize', 10, 'linewidth', 2);
end
grid on
box on
%% Export the problem configuration and solutions
writematrix([path', curv', goals', endpts'],'sequence.csv');
% save('full_range_centered','p_vals','Pi', 'goals', 'results', 'path', 'curv', 'lb', 'ub', 'radial_constraint');
%% 
% PLOTTING

%% Format solution figures for report - position
box on
xlim([-0.75,0.75])
ylim([-0.05,0.85])
xlabel('$x_B$ (m)','Interpreter','latex')
ylabel('$y_B$ (m)','Interpreter','latex')
yticklabels('auto')
xticks([-0.7,-0.5,-0.3,-0.1,0.1,0.3,0.5,0.7])
yticks([0,0.2,0.4,0.6,0.8])
ax = gca;
set(ax, 'FontSize', 40)
set(ax, 'TickLabelInterpreter', 'latex')
% exportgraphics(ax,'D:\Study\Thesis\Report\Rewrite\images\updated_figures\endpoint_sols\black_weighted\05.eps')

%% Format solution figures for report - orientation
xlim([-0.5,0.5])
ylim([0,0.9])
xticks(-0.6:0.2:0.6)
yticks(0:0.1:0.9)
xlabel('$x_B$ (m)','Interpreter','latex')
ylabel('$y_B$ (m)','Interpreter','latex')
ax = gca;
set(ax, 'FontSize', 24)
set(ax, 'TickLabelInterpreter', 'latex')
%exportgraphics(ax,'D:\Study\Thesis\Report\images\orientation_analysis\orange_orientation_solutions.eps')

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
    plot_config(q_i,1)%goals(2,i)/goals(2,end))
    hold on
    endpt = fk_fcn(p_vals, q_i, 1, 0);
    endpts = [endpts endpt];
    plot([endpt(1) goals(1,i)], [endpt(2) goals(2,i)],'k:')
end
%plot(goals(1,:),goals(2,:),Color=[0.75 0.75 1.0])
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
    obj = norm(diag([1e2 1e2 1e0])*(fka_fcn(p_vals, q, 1, 0) - goal));
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


%%
clear
clear global
rmpath('../fixed_base/automatically_generated')
addpath('automatically_generated')
global k_obj K p_vals Theta_bar

%% Object properties
% Load predefined object parameters
load('../object_parameters/black_short_weighted.mat')

% % Manually defined object parameters (overwrites loaded parameters)
% p_vals = [0.6, 0.23, 0.6, 0.02]';
% k_obj = 0.1799;
% Theta_bar = [-0.0463, 1.3731];

Pi = [k_obj; Theta_bar];
H = [1, 1/2; 1/2, 1/3];
K = k_obj*H;

%%
% Cost function characterisation
global theta0_grid
theta0_grid = -6:0.1:-1e-3;
global theta1_grid
theta1_grid = 1e-3:0.1:8;
global x_grid
x_grid = -0.0:0.1:0.0;
global z_grid
z_grid = 0.7:0.1:0.7;
global phi_grid
phi_grid = (pi/180)*(0:1:135);

goal_calc = [0;0.35];
global cost_grid
cost_grid = Inf*ones(length(theta0_grid),length(theta1_grid),length(x_grid),length(z_grid),length(phi_grid));
model_constraint_violated = false;

theta0_idx = 1;
for theta0_calc = theta0_grid
    theta0_calc
    theta1_idx = 1;
    for theta1_calc = theta1_grid
        phi_idx = 1;
        for phi_calc = phi_grid
            x_idx = 1;
            for x_calc = x_grid
                z_idx = 1;
                for z_calc = z_grid
                    q_calc = [theta0_calc;theta1_calc;x_calc;z_calc;phi_calc];
                    G_eval = G_fcn(p_vals,q_calc);
                    if all(abs((G_eval(1:2) + K*[theta0_calc-Theta_bar(1); theta1_calc-Theta_bar(2)])) < 1e-6) % 1e-6 is default fmincon constraint tolerance
                        cost_grid(theta0_idx,theta1_idx,x_idx,z_idx,phi_idx) = norm(fk_fcn(p_vals, q_calc, 1, 0) - goal_calc);
                    else
                        % If the model constraint is violated we don't need to check any other X/Z points for the same Theta/Phi combination
                        model_constraint_violated = true;
                        break;
                    end
                    z_idx = z_idx + 1;
                end
                if model_constraint_violated == true
                    model_constraint_violated = false;
                    break;
                end
                x_idx = x_idx + 1;
            end
            phi_idx = phi_idx + 1;
        end
        theta1_idx = theta1_idx + 1;
    end
    theta0_idx = theta0_idx + 1;
end

%% Save cost grid
save('cost_grid_check_model_const_1deg_tenthTheta', 'p_vals', 'k_obj', 'Theta_bar', 'goal_calc', ...
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
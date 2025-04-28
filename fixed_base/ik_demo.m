clear
clear global
rmpath('automatically_generated')
rmpath('auto_gen_polynomial_1')
rmpath('auto_gen_polynomial_2')

% Change degree here
degree = 1;

fcn_dir = "auto_gen_polynomial_" + degree
addpath(fcn_dir)

global k_obj K p_vals Theta_bar
global beta_obj D

load('../object_parameters/black_weighted.mat')

% Plot arbitrary fk - all thetas 1
plot_config_polynomial(ones(degree+1,1),1,0)
title('FK')

% Inverse kinematics using fk targets from above plot, with initial guess of almost zero curvature
theta_out = ik(p_vals, 1e-3*ones(degree+1,1), [-0.085,-0.283,-0.336,-0.429]', 0.01, 10)

figure
plot_config_polynomial(theta_out,1,0)
title('IK')
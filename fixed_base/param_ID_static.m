clear
clear global
rmpath('../floating_base/automatically_generated')
addpath('automatically_generated')

%%
% Load predefined object parameters
load('../object_parameters/full_dynamic_id/black_short_loop_100g.mat')

%%
% Load data
data = readmatrix("data_in/0402-loop_static_id/100g/theta_equilibria.csv"); % TODO - make this import as column vectors with inherited names
range = [1,length(Gamma)];
num_samples = range(2) - range(1) + 1;
Gamma_set = -Gamma(range(1):range(2)); % Note - use -ve Gamma since data is robot angle
Theta_set = [Theta0(range(1):range(2))'; Theta1(range(1):range(2))'];

%%
% Joint identification of k and theta_bar
clear delta Y
for sample = 1:num_samples

    RHS = -Gv_fcn(p_vals,Theta_set(:,sample),Gamma_set(sample));
    Y_n = [Theta_set(1,sample)+Theta_set(2,sample)/2   -1   -1/2;
           Theta_set(1,sample)/2+Theta_set(2,sample)/3 -1/2 -1/3];
    
    if sample == 1
        delta = RHS;
        Y = Y_n;
    else
        delta = [delta; RHS];
        Y = [Y; Y_n];
    end

end

Pi = pinv(Y)*delta;
k_id = Pi(1)
theta_bar_0_id = Pi(2)/k_id
theta_bar_1_id = Pi(3)/k_id

%% Save parameters
k_obj = k_id;
Theta_bar = [theta_bar_0_id; theta_bar_1_id];
save('black_short_loop_100g_static', 'p_vals', 'k_obj', 'Theta_bar')

%%
% % Theta_bar from static eqns with variable gravity field and prescribed K
% K_known = 1e-1*[1, 1/2; 1/2, 1/3];
% G_scale = 1.0;
% clear delta Y
% 
% for sample = 1:num_samples
% 
%     %   RHS = -G(Theta,Gamma) -K*Theta
%     RHS = -G_scale*Gv_fcn(p_vals,Theta_set(:,sample),Gamma_set(sample)) -K_known*Theta_set(:,sample);
%     Y_n = -K_known*Theta_set(sample);  
%     
%     if sample == 1
%         delta = RHS;
%         Y = Y_n;
%     else
%         delta = [delta; RHS];
%         Y = [Y; Y_n];
%     end
% 
% end
% 
% Pi = pinv(Y)*delta
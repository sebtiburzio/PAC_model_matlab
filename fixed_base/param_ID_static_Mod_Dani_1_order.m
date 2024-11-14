clear
clear global
% rmpath('automatically_generated')
addpath('automatically_generated')

%%
% Load predefined object parameters
load('../object_parameters/full_dynamic_id/black_weighted_dyn.mat')

%%
% Load data
data = readmatrix("data_in/0802-equilibrium_data\black_weighted_equilibria\theta_equilibria.csv"); % TODO - make this import as column vectors with inherited names
% range = [1,length(Gamma)];
range = [1,size(data,1)];
num_samples = range(2) - range(1) + 1;
Gamma = data(:,1);
Theta0 = data(:,2);
Theta1 = data(:,3);
X_mid = data(:,4);
Z_mid = data(:,5);
X_end = data(:,6);
Z_end = data(:,7);

Gamma_set = -Gamma(range(1):range(2)); % Note - use -ve Gamma since data is robot angle
Theta_set = [Theta0(range(1):range(2))'; Theta1(range(1):range(2))'];
Pos_mid_set = [X_mid';Z_mid'];
Pos_end_set = [X_mid';Z_mid'];

%%
%Joint identification of k and theta_bar

% clear delta Y
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
k_id = Pi(1);
theta_bar_0_id = Pi(2)/k_id
theta_bar_1_id = Pi(3)/k_id

%% Save parameters
k_obj = k_id;
Theta_bar = [theta_bar_0_id; theta_bar_1_id];
save('black_short_loop_100g_static', 'p_vals', 'k_obj', 'Theta_bar')

% return

options = optimset('PlotFcns',{@optimplotfval,@optimplotx});

fun = @FitShapeGivenE;
E = fminsearch(fun,[k_obj,Theta_bar(1),Theta_bar(2)],options);


%%%%% OPTIMIZER

function cost = FitShapeGivenE(x)

load('../object_parameters/full_dynamic_id/black_weighted_dyn.mat')

%%
% Load data
data = readmatrix("data_in/0802-equilibrium_data\black_weighted_equilibria\theta_equilibria.csv"); % TODO - make this import as column vectors with inherited names
% range = [1,length(Gamma)];
range = [1,size(data,1)];
num_samples = range(2) - range(1) + 1;
Gamma = data(:,1);
Theta0 = data(:,2);
Theta1 = data(:,3);
X_mid = data(:,4);
Z_mid = data(:,5);
X_end = data(:,6);
Z_end = data(:,7);

Gamma_set = -Gamma(range(1):range(2)); % Note - use -ve Gamma since data is robot angle
Theta_set = [Theta0(range(1):range(2))'; Theta1(range(1):range(2))'];
Pos_mid_set = [X_mid';Z_mid'];
Pos_end_set = [X_end';Z_end'];

k_obj = x(1);
theta_bar = [x(2);x(3)];

% k_obj = 0.1414;
% theta_bar =[0.5642;0.5978];

% Gamma_set = -Gamma(range(1):range(2)); % Note - use -ve Gamma since data is robot angle
% Theta_set = [Theta0(range(1):range(2))'; Theta1(range(1):range(2))'];
% Pos_mid_set = [X_mid';Z_mid'];
% Pos_end_set = [X_mid';Z_mid'];

H = [1 1/2;1/2 1/3];

% I CHECK FOR ALL THE CASES

Pos_mid_est_all = [];
Pos_end_est_all = [];

    for i = 1:length(Gamma_set)

        % i=15;
        Gamma_here       = Gamma_set(i);
        Theta_set_here   = Theta_set(:,i);        % I USE THIS AS AN INITIAL GUESS
    
        %%%%%%%%%%%%%%%%%%%%%% SOLVE STATIC EQUILIBRIUM %%%%%%%%%%%%%%%%%%%%%%
    
        ini_guess2         =Theta_set_here;
        options2           =optimoptions('lsqnonlin','Display','iter','MaxFunctionEvaluations',200000,'MaxIterations',10000,'FunctionTolerance',1e-8,'OptimalityTolerance',1e-6);
        fun2               =@(z)pwvariable_Static_Equilibrium(z,p_vals,Gamma_here,H,k_obj,theta_bar);
        LB                 =[ones(length(ini_guess2),1)*-inf];
        UB                 =[ones(length(ini_guess2),1)*inf];
        z                  =lsqnonlin(fun2,ini_guess2,LB,UB,options2);
        % z = Theta_set_here;

        % plot_config(z(1),z(2),1,-Gamma_here*1);hold on;plot(Pos_mid_set(1,i),Pos_mid_set(2,i),'o');hold on;plot(Pos_end_set(1,i),Pos_end_set(2,i),'o')
    
        s_vect = 0:0.05:1;
        xy_c = nan(length(s_vect),2);
        for i_s = 1:length(s_vect)
            xy_c(i_s,:) = fk_fcn(p_vals, [z(1),z(2)]', s_vect(i_s), 0);
        end

        Gamma_here2 = -Gamma_here*1;
        xy_c = ([cos(Gamma_here2) sin(Gamma_here2); -sin(Gamma_here2) cos(Gamma_here2)]*xy_c')';
    
        % For plotting final shapes
        % Gamma_here2 = -Gamma_here;
        alpha =1;
        hold on;plot(xy_c(:,1),xy_c(:,2), Color=[1.0 1.0-alpha*0.5 1.0-alpha*1.0], LineWidth=3)
        hold on;
        % hold on;plot(Pos_mid_set(1,i),Pos_mid_set(2,i),'o');hold on;plot(Pos_end_set(1,i),Pos_end_set(2,i),'o')
        % plot([-0.05,0.05],[0,0],'k',LineWidth=2)
        scatter(xy_c(11,1),xy_c(11,2), 30, [0.4660 0.6740 0.1880], 'filled')
        scatter(xy_c(end,1),xy_c(end,2), 30, [0 0.4470 0.7410], 'filled')
        xlim([-p_vals(3)-0.1,p_vals(3)+0.1])
        axis equal
        xlabel('x (m)')
        ylabel('y (m)')
        grid on
        
        ind_middle = find(min(abs(s_vect-0.5))==abs(s_vect-0.5));
        Pos_mid_est = xy_c(ind_middle,:);
        Pos_mid_est_all = [Pos_mid_est_all Pos_mid_est'];
    
        Pos_end_est = xy_c(end,:);
        Pos_end_est_all = [Pos_end_est_all Pos_end_est'];
    end

    Error_Pos_End = Pos_end_set-Pos_end_est_all;
    Error_Pos_Mid = Pos_mid_set-Pos_mid_est_all;
    cost = sum(sqrt(sum(Error_Pos_End.^2,1)))+sum(sqrt(sum(Error_Pos_Mid.^2,1)));

end

subplot(2,1,1);plot(Gamma_set, sqrt(sum(Error_Pos_Mid.^2,1)),'o',LineWidth=2);
xlabel('\phi (rad)');
ylabel('\Delta p_{m} (m)');grid on;

% Etiquetas en múltiplos de pi en el eje x
xticks([-pi, -3*pi/4, -pi/2, -pi/4, 0, pi/4, pi/2, 3*pi/4, pi]);  % Múltiplos de pi que quieres mostrar
xticklabels({'-\pi','-3\pi/4','-\pi/2','-\pi/4' '0','\pi/4' '\pi/2','3\pi/4' ,'\pi'});  % Etiquetas en función de pi
set(gca, 'FontSize', 22)

subplot(2,1,2);plot(Gamma_set, sqrt(sum(Error_Pos_End.^2,1)),'o',LineWidth=2);
xlabel('\phi (rad)');
ylabel('\Delta p_{e} (m)');grid on;

% Etiquetas en múltiplos de pi en el eje x
xticks([-pi, -3*pi/4, -pi/2, -pi/4, 0, pi/4, pi/2, 3*pi/4, pi]);  % Múltiplos de pi que quieres mostrar
xticklabels({'-\pi','-3\pi/4','-\pi/2','-\pi/4' '0','\pi/4' '\pi/2','3\pi/4' ,'\pi'});  % Etiquetas en función de pi
set(gca, 'FontSize', 22)

return




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
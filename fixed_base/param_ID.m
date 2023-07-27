data = readmatrix("data_in/0619/orange_swing/theta_evolution.csv");
ts = data(:,1)';
Theta0 = data(:,2)';
Theta1 = data(:,3)';
dTheta0 = data(:,4)';
dTheta1 = data(:,5)';
ddTheta0 = data(:,6)';
ddTheta1 = data(:,7)';

num_samples = length(ts);

Theta = [Theta0; Theta1];
dTheta = [dTheta0; dTheta1];
ddTheta = [ddTheta0; ddTheta1];

% coefficients for unknown beta
c0 = dTheta0 + dTheta1/2;
c1 = dTheta0/2 + dTheta1/3;

% coefficeints for unknown k & beta
c00 = Theta0 + Theta1/2.0;
c01 = dTheta0 + dTheta1/2.0;
c10 = Theta0/2.0 + Theta1/3.0;
c11 = dTheta0/2.0 + dTheta1/3.0;

%% Identify stiffness and damping
% # Calc delta vector
for sample = 1:num_samples
%     if mod(sample,10) == 0
%         sample
%     end

    %   RHS = 0 - B*ddQ - C*dQ - G 
    RHS = - B_fcn(p_vals,Theta(:,sample))*ddTheta(:,sample) ...
            - C_fcn(p_vals,Theta(:,sample),dTheta(:,sample))*dTheta(:,sample) ...
            - G_fcn(p_vals,Theta(:,sample));
    
    if sample == 1
        delta = RHS;
    else
        delta = [delta; RHS];
    end

end

for sample = 1:num_samples
    
    Y_n = [c00(sample),c01(sample);
           c10(sample),c11(sample)];

    if sample == 1
        Y = Y_n;
    else
        Y = [Y; Y_n];
    end

end

%% Identify cable mass, stiffness and damping
% # Calc delta vector & Y matrix
for sample = 1:num_samples

    RHS = - E_mL_0(p_vals,Theta(:,sample),dTheta(:,sample),ddTheta(:,sample));
    
    if sample == 1
        delta = RHS;
    else
        delta = [delta; RHS];
    end
    
    Y_n = [c00(sample),c01(sample);
           c10(sample),c11(sample)];
    Y_n = [Y_n, dE_dmL(p_vals,Theta(:,sample),dTheta(:,sample),ddTheta(:,sample))];

    if sample == 1
        Y = Y_n;
    else
        Y = [Y; Y_n];
    end

end

%% Identify end mass, stiffness and damping
% # Calc delta vector & Y matrix
for sample = 1:num_samples

    RHS = - E_mE_0(p_vals,Theta(:,sample),dTheta(:,sample),ddTheta(:,sample));
    
    if sample == 1
        delta = RHS;
    else
        delta = [delta; RHS];
    end
    
    Y_n = [c00(sample),c01(sample);
           c10(sample),c11(sample)];
    Y_n = [Y_n, dE_dmE(p_vals,Theta(:,sample),dTheta(:,sample),ddTheta(:,sample))];

    if sample == 1
        Y = Y_n;
    else
        Y = [Y; Y_n];
    end

end

%% Identify B/C Scale and Beta
p_vals = [0.4, 0.23, 0.75, 0.015]';

k = 0.0819;
Theta_bar = [-1.2516; 0.9568];

num_samples = length(ts);

for sample = 1:num_samples
    
    RHS = -G_fcn(p_vals, Theta(:,sample)) - k*[1 1/2; 1/2 1/3]*(Theta(:,sample)-Theta_bar);

    Y_n = [
            B_fcn(p_vals,Theta(:,sample))*ddTheta(:,sample) + C_fcn(p_vals,Theta(:,sample),dTheta(:,sample))*dTheta(:,sample), ...
            [c0(sample); c1(sample)] 
          ];
    
    if sample == 1
        delta = RHS;
        Y = Y_n;
    else
        delta = [delta; RHS];
        Y = [Y; Y_n];
    end

end

Pi = pinv(Y)*delta

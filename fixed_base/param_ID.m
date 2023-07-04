data = readmatrix("black_swing.csv");
t = data(:,1)';
Theta0 = data(:,2)';
Theta1 = data(:,3)';
dTheta0 = data(:,4)';
dTheta1 = data(:,5)';
ddTheta0 = data(:,6)';
ddTheta1 = data(:,7)';

p_vals = [0.6, 0.23, 0.61, 0.012]';

num_samples = length(t);

%% Offset Theta
Theta0 = Theta0 - mean(Theta0(end-30:end));
Theta1 = Theta1 - mean(Theta1(end-30:end));

%%
Theta = [Theta0; Theta1];
dTheta = [dTheta0; dTheta1];
ddTheta = [ddTheta0; ddTheta1];

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

%%
Pi = pinv(Y)*delta
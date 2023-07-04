%%
% Constant parameters
syms m_L m_E L D real % m_L - total mass of cable, m_E - mass of weighted end
p = [m_L;m_E;L;D];
num_masses = 6;  % Number of masses to discretise along length (not including end mass)
syms gamma % Gravity direction

% Configuration variables
syms theta_0 theta_1 dtheta_0 dtheta_1 ddtheta_0 ddtheta_1
theta = [theta_0;theta_1];
dtheta = [dtheta_0;dtheta_1];
ddtheta = [ddtheta_0;ddtheta_1];

% Object coordinates in global frame (forward kinematics)
fk_fcn = sym(zeros(2,1));
syms alpha real % tip orientation in object base frame

% Integration variables
syms s v d real

%% 
% Forward Kinematics
tic

% Spine x,z in object base frame, defined as if it was reflected in the robot XY plane
alpha = -(theta_0*v + 0.5*theta_1*v^2); % negative curvature so sense matches robot frame Y axis rotation
fk_fcn(1) = L*int(sin(alpha),v, 0, s); % x. when theta=0, x=0.
fk_fcn(2) = -L*int(cos(alpha),v, 0, s); % z. when theta=0, z=-L. 

rot_alpha = [cos(subs(alpha,v,s)) sin(subs(alpha,v,s));
            -sin(subs(alpha,v,s)) cos(subs(alpha,v,s))]; 
fk_fcn = fk_fcn + D*rot_alpha*[0; d];

toc

% Export FK function
matlabFunction(fk_fcn,'File','automatically_generated/fixed/fk_fcn','Vars',{p, theta, s, d}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/fk_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/fk_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Potential (gravity) vector
tic

% Energy
U = m_E*int(subs(sin(gamma)*fk_fcn(1) + cos(gamma)*fk_fcn(2),s,1),d,-1/2,1/2); 
for i = 0:num_masses-1
    U = U + (m_L/num_masses)*int(subs(sin(gamma)*fk_fcn(1) + cos(gamma)*fk_fcn(2),s,i/num_masses + 1/(num_masses*2)),d,-1/2,1/2);
end

% Potential force
G_fcn = jacobian(9.81*(subs(U,gamma,0)),[theta_0; theta_1])'; % Gravity field Z direction
Gv_fcn = jacobian(9.81*(U),[theta_0; theta_1])'; % Variable gravity field

toc

matlabFunction(G_fcn,'File','automatically_generated/fixed/G_fcn','Vars',{p, theta}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/G_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/G_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

matlabFunction(Gv_fcn,'File','automatically_generated/fixed/Gv_fcn','Vars',{p, theta, gamma}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/Gv_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/Gv_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Inertia matrix
tic

J = jacobian(subs(fk_fcn,s, 1),[theta_0; theta_1]);
B_fcn = 0.5*m_E*int(J'*J, d, -1/2, 1/2);
for i = 0:num_masses-1
    J = jacobian(subs(fk_fcn, s, i/num_masses + 1/(num_masses*2)),[theta_0; theta_1]);
    B_fcn = B_fcn + 0.5*(m_L/num_masses)*int(J'*J, d, -1/2, 1/2);
end

toc

matlabFunction(B_fcn,'File','automatically_generated/fixed/B_fcn','Vars',{p, theta}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/B_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/B_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Centrifugal / Coriolis matrix
tic

C_fcn = sym(zeros(2));
for i = 1:2
    for j = 1:2
        for k = 1:2
            Christoffel = 0.5*(diff(B_fcn(i,j),theta(k)) + diff(B_fcn(i,k),theta(j)) - diff(B_fcn(j,k),theta(i)));
            C_fcn(i,j) = C_fcn(i,j) + Christoffel*dtheta(k);
        end
    end
end

toc

matlabFunction(C_fcn,'File','automatically_generated/fixed/C_fcn','Vars',{p,theta,dtheta}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/C_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/C_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Factorisation by masses
E = B_fcn*ddtheta + C_fcn*dtheta + G_fcn;
Y = jacobian(E,[m_L;m_E]);

%%
% Factor out m_L for identification
dE_dmL = diff(E,m_L);
E_mL_0 = subs(E,m_L,0);

matlabFunction(dE_dmL,'File','automatically_generated/fixed/dE_dmL','Vars',{p,theta,dtheta,ddtheta}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/dE_dmL.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/dE_dmL.m','w');
fprintf(fid,'%s',f);
fclose(fid);

matlabFunction(E_mL_0,'File','automatically_generated/fixed/E_mL_0','Vars',{p,theta,dtheta,ddtheta}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/E_mL_0.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/E_mL_0.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Factor out m_E for identification
dE_dmE = diff(E,m_E);
E_mE_0 = subs(E,m_E,0);

matlabFunction(dE_dmE,'File','automatically_generated/fixed/dE_dmE','Vars',{p,theta,dtheta,ddtheta}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/dE_dmE.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/dE_dmE.m','w');
fprintf(fid,'%s',f);
fclose(fid);

matlabFunction(E_mE_0,'File','automatically_generated/fixed/E_mE_0','Vars',{p,theta,dtheta,ddtheta}); % creating the MatLab function

fid  = fopen('automatically_generated/fixed/E_mE_0.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fixed/E_mE_0.m','w');
fprintf(fid,'%s',f);
fclose(fid);

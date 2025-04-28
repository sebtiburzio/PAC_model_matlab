clear
clear global

degree = 2; % Change to generate equations for different polynomal representations
dirname = "auto_gen_polynomial_" + degree + '/';
mkdir(dirname)

%%
% Constant parameters
syms m_L m_E L D real % m_L - total mass of cable, m_E - mass of weighted end
p = [m_L;m_E;L;D];
num_masses = 6;  % Number of masses to discretise along length (not including end mass)
syms gamma % Gravity direction

% Configuration variables
syms theta [degree+1 1]
syms dtheta [degree+1 1]

% Object coordinates in global frame (forward kinematics)
fk_fcn = sym(zeros(2,1));
syms alpha real % tip orientation in object base frame

% Integration variables
syms s v d real

%% 
% Forward Kinematics
tic

% Spine x,z in object base frame, defined as if it was reflected in the robot XY plane
alpha = 0;
for n = 1:degree+1
    alpha = alpha + 1/factorial(n)*theta(n)*v^n;
end
fk_fcn(1) = -L*int(sin(alpha),v, 0, s); % x. when theta=0, x=0.
fk_fcn(2) = -L*int(cos(alpha),v, 0, s); % z. when theta=0, z=-L. 

rot_alpha = [cos(subs(alpha,v,s)) sin(subs(alpha,v,s)); % +ve rotations around robot base Y axis (CW in XZ plane)
             -sin(subs(alpha,v,s)) cos(subs(alpha,v,s))]; 
fk_fcn = fk_fcn + D*rot_alpha*[d; 0];

toc

% Export FK function
matlabFunction(fk_fcn,'File','./' + dirname + 'fk_fcn','Vars',{p, theta, s, d}); % creating the MatLab function

fid  = fopen('./' + dirname + 'fk_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('./' + dirname + 'fk_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Jacobian for IK

J = jacobian(fk_fcn,theta);

matlabFunction(J,'File','./' + dirname + 'J_fcn','Vars',{p, theta, s, d}); % creating the MatLab function

fid  = fopen('./' + dirname + 'J_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('./' + dirname + 'J_fcn.m','w');
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
G_fcn = jacobian(9.81*(subs(U,gamma,0)),theta)'; % Gravity field Z direction
Gv_fcn = jacobian(9.81*(U),theta)'; % Variable gravity field

toc

matlabFunction(G_fcn,'File','./' + dirname + 'G_fcn','Vars',{p, theta}); % creating the MatLab function

fid  = fopen('./' + dirname + 'G_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('./' + dirname + 'G_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

matlabFunction(Gv_fcn,'File','./' + dirname + 'Gv_fcn','Vars',{p, theta, gamma}); % creating the MatLab function

fid  = fopen('./' + dirname + 'Gv_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('./' + dirname + 'Gv_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Inertia matrix and Jacobian
tic

J = jacobian(subs(fk_fcn,s, 1),theta);
B_fcn = m_E*int(J'*J, d, -1/2, 1/2);
for i = 0:num_masses-1
    J = jacobian(subs(fk_fcn, s, i/num_masses + 1/(num_masses*2)),theta);
    B_fcn = B_fcn + (m_L/num_masses)*int(J'*J, d, -1/2, 1/2);
end

toc

matlabFunction(B_fcn,'File','./' + dirname + 'B_fcn','Vars',{p, theta}); % creating the MatLab function

fid  = fopen('./' + dirname + 'B_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('./' + dirname + 'B_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Centrifugal / Coriolis matrix
tic

C_fcn = sym(zeros(degree+1));
for i = 1:degree+1
    for j = 1:degree+1
        for k = 1:degree+1
            Christoffel = 0.5*(diff(B_fcn(i,j),theta(k)) + diff(B_fcn(i,k),theta(j)) - diff(B_fcn(j,k),theta(i)));
            C_fcn(i,j) = C_fcn(i,j) + Christoffel*dtheta(k);
        end
    end
end

toc

matlabFunction(C_fcn,'File','./' + dirname + 'C_fcn','Vars',{p,theta,dtheta}); % creating the MatLab function

fid  = fopen('./' + dirname + 'C_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('./' + dirname + 'C_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

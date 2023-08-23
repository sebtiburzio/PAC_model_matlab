%%
% Constant parameters
syms m_L m_E L D real % m_L - total mass of cable, m_E - mass of weighted end
p = [m_L;m_E;L;D];
num_masses = 6;  % Number of masses to discretise along length (not including end mass)
syms gamma % Gravity direction

% Configuration variables
syms theta_0 theta_1 x z phi real
syms dtheta_0 dtheta_1 dx dz dphi real
q = [theta_0;theta_1;x;z;phi];
dq = [dtheta_0;dtheta_1;dx;dz;dphi];

% Object coordinates in global frame (forward kinematics)
fk_fcn = sym(zeros(2,1));
syms alpha real % tip orientation in object base frame

% Integration variables
syms s v d real

%% 
% Forward Kinematics
tic

% Spine x,z in object base frame, defined as if it was reflected in the robot XY plane
alpha = theta_0*v + 0.5*theta_1*v^2;
fk_fcn(1) = -L*int(sin(alpha),v, 0, s); % x. when theta=0, x=0.
fk_fcn(2) = -L*int(cos(alpha),v, 0, s); % z. when theta=0, z=-L. 

% FK of midpoint and endpoint in base frame (for curvature IK)
%fk_mid_fixed = subs(fk, s, 0.5);
%fk_end_fixed = subs(fk, s, 1);
%J_mid_fixed = jacobian(fk_mid_fixed,[theta_0; theta_1]);
%J_end_fixed = jacobian(fk_end_fixed,[theta_0; theta_1]);

% 3DOF floating base 
rot_phi = [cos(phi) sin(phi); % +ve rotations around robot base Y axis (CW in XZ plane)
          -sin(phi) cos(phi)];                  
rot_alpha = [cos(subs(alpha,v,s)) sin(subs(alpha,v,s));
             -sin(subs(alpha,v,s)) cos(subs(alpha,v,s))]; 
fk_fcn = [x; z] + rot_phi*(fk_fcn + D*rot_alpha*[d; 0]);

fka_fcn = [fk_fcn; phi + subs(alpha,v,s)];

toc

% Export FK function
matlabFunction(fk_fcn,'File','automatically_generated/fk_fcn','Vars',{p, q, s, d}); % creating the MatLab function

fid  = fopen('automatically_generated/fk_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fk_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

% With orientation
matlabFunction(fka_fcn,'File','automatically_generated/fka_fcn','Vars',{p, q, s, d}); % creating the MatLab function

fid  = fopen('automatically_generated/fka_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/fka_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Potential (gravity) vector
tic

% Energy
% TODO - add base mass
U = m_E*int(subs(sin(gamma)*fk_fcn(1) + cos(gamma)*fk_fcn(2),s,1),d,-1/2,1/2); 
for i = 0:num_masses-1
    U = U + (m_L/num_masses)*int(subs(sin(gamma)*fk_fcn(1) + cos(gamma)*fk_fcn(2),s,i/num_masses + 1/(num_masses*2)),d,-1/2,1/2);
end

% Potential force
G_fcn = jacobian(9.81*(subs(U,gamma,0)),[theta_0; theta_1; x; z; phi])'; % Gravity field Z direction
Gv_fcn = jacobian(9.81*(U),[theta_0; theta_1; x; z; phi])'; % Variable gravity field

toc

matlabFunction(G_fcn,'File','automatically_generated/G_fcn','Vars',{p, q}); % creating the MatLab function

fid  = fopen('automatically_generated/G_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/G_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

matlabFunction(Gv_fcn,'File','automatically_generated/Gv_fcn','Vars',{p, q, gamma}); % creating the MatLab function

fid  = fopen('automatically_generated/Gv_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/Gv_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Inertia matrix
tic

% TODO - add base mass
J = jacobian(subs(fk_fcn,s, 1),[theta_0; theta_1; x; z; phi]);
B_fcn = m_E*int(J'*J, d, -1/2, 1/2);
for i = 0:num_masses-1
    J = jacobian(subs(fk_fcn, s, i/num_masses + 1/(num_masses*2)),[theta_0; theta_1; x; z; phi]);
    B_fcn = B_fcn + (m_L/num_masses)*int(J'*J, d, -1/2, 1/2);
end

toc

matlabFunction(B_fcn,'File','automatically_generated/B_fcn','Vars',{p, q}); % creating the MatLab function

fid  = fopen('automatically_generated/B_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/B_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Centrifugal / Coriolis matrix
tic

C_fcn = sym(zeros(5));
for i = 1:5
    for j = 1:5
        for k = 1:5
            Christoffel = 0.5*(diff(B_fcn(i,j),q(k)) + diff(B_fcn(i,k),q(j)) - diff(B_fcn(j,k),q(i)));
            C_fcn(i,j) = C_fcn(i,j) + Christoffel*dq(k);
        end
    end
end

toc

matlabFunction(C_fcn,'File','automatically_generated/C_fcn','Vars',{p, q, dq}); % creating the MatLab function

fid  = fopen('automatically_generated/C_fcn.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/C_fcn.m','w');
fprintf(fid,'%s',f);
fclose(fid);
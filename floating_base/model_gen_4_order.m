clear
clear global

%%
% Constant parameters
syms m_L m_E L D real % m_L - total mass of cable, m_E - mass of weighted end
p = [m_L;m_E;L;D];
num_masses = 6;  % Number of masses to discretise along length (not including end mass)
syms gamma % Gravity direction

% Configuration variables for second order polynomial
syms theta_0 theta_1 theta_2 theta_3 theta_4 x z phi real
syms dtheta_0 dtheta_1 dtheta_2 dtheta_3 dtheta_4 dx dz dphi real
q = [theta_0;theta_1;theta_2;theta_3;theta_4;x;z;phi];
dq = [dtheta_0;dtheta_1;dtheta_2;dtheta_3;dtheta_4;dx;dz;dphi];

% Object coordinates in global frame (forward kinematics)
fk_fcn = sym(zeros(2,1));
syms alpha real % tip orientation in object base frame

% Integration variables
syms s v d real

%% 
% Forward Kinematics
tic

% Spine x,z in object base frame, defined as if it was reflected in the robot XY plane
alpha = theta_0*v + 0.5*theta_1*v^2 + 1/3*theta_2*v^3 + 1/4*theta_3*v^4 + 1/5*theta_4*v^5;
fk_fcn(1) = -L*int(sin(alpha),v, 0, s); % x. when theta=0, x=0.
fk_fcn(2) = -L*int(cos(alpha),v, 0, s); % z. when theta=0, z=-L. 

% 3DOF floating base 
rot_phi = [cos(phi) sin(phi); % +ve rotations around robot base Y axis (CW in XZ plane)
          -sin(phi) cos(phi)];                  
rot_alpha = [cos(subs(alpha,v,s)) sin(subs(alpha,v,s));
             -sin(subs(alpha,v,s)) cos(subs(alpha,v,s))]; 
fk_fcn = [x; z] + rot_phi*(fk_fcn + D*rot_alpha*[d; 0]);

fka_fcn = [fk_fcn; phi + subs(alpha,v,s)];

toc

% Export FK function
matlabFunction(fk_fcn,'File','automatically_generated/poly_order_4/fk_fcn_order_4','Vars',{p, q, s, d}); % creating the MatLab function

fid  = fopen('automatically_generated/poly_order_4/fk_fcn_order_4.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/poly_order_4/fk_fcn_order_4.m','w');
fprintf(fid,'%s',f);
fclose(fid);

% With orientation
matlabFunction(fka_fcn,'File','automatically_generated/poly_order_4/fka_fcn_order_4','Vars',{p, q, s, d}); % creating the MatLab function

fid  = fopen('automatically_generated/poly_order_4/fka_fcn_order_4.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/poly_order_4/fka_fcn_order_4.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Potential (gravity) vector
tic

% Energy
U = 0.08*int(subs(sin(gamma)*fk_fcn(1) + cos(gamma)*fk_fcn(2),s,0),d,-1/2,1/2); % Base mass currently just FT sensor flange mass. Should be combined with cable clamp and adapter (these are currently included in cable weight).
U = U + m_E*int(subs(sin(gamma)*fk_fcn(1) + cos(gamma)*fk_fcn(2),s,1),d,-1/2,1/2); 
for i = 0:num_masses-1
    U = U + (m_L/num_masses)*int(subs(sin(gamma)*fk_fcn(1) + cos(gamma)*fk_fcn(2),s,i/num_masses + 1/(num_masses*2)),d,-1/2,1/2);
end

% Potential force
G_fcn = jacobian(9.81*(subs(U,gamma,0)),[theta_0; theta_1; theta_2; theta_3; theta_4; x; z; phi])'; % Gravity field Z direction
Gv_fcn = jacobian(9.81*(U),[theta_0; theta_1; theta_2; theta_3; theta_4; x; z; phi])'; % Variable gravity field

toc

matlabFunction(G_fcn,'File','automatically_generated/poly_order_4/G_fcn_order_4','Vars',{p, q}); % creating the MatLab function

fid  = fopen('automatically_generated/poly_order_4/G_fcn_order_4.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/poly_order_4/G_fcn_order_4.m','w');
fprintf(fid,'%s',f);
fclose(fid);

matlabFunction(Gv_fcn,'File','automatically_generated/poly_order_4/Gv_fcn_order_4','Vars',{p, q, gamma}); % creating the MatLab function

fid  = fopen('automatically_generated/poly_order_4/Gv_fcn_order_4.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/poly_order_4/Gv_fcn_order_4.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Inertia matrix
tic

J = jacobian(subs(fk_fcn,s, 0),[theta_0; theta_1; theta_2; theta_3; theta_4; x; z; phi]);
B_fcn = 0.08*int(J'*J, d, -1/2, 1/2); % Base mass currently just FT sensor flange mass. Should be combined with cable clamp and adapter (these are currently included in cable weight).
J = jacobian(subs(fk_fcn,s, 1),[theta_0; theta_1; theta_2; theta_3; theta_4;x; z; phi]);
B_fcn = B_fcn + m_E*int(J'*J, d, -1/2, 1/2);
for i = 0:num_masses-1
    J = jacobian(subs(fk_fcn, s, i/num_masses + 1/(num_masses*2)),[theta_0; theta_1; theta_2; theta_3; theta_4; x; z; phi]);
    B_fcn = B_fcn + (m_L/num_masses)*int(J'*J, d, -1/2, 1/2);
end

toc

matlabFunction(B_fcn,'File','automatically_generated/poly_order_4/B_fcn_order_4','Vars',{p, q}); % creating the MatLab function

fid  = fopen('automatically_generated/poly_order_4/B_fcn_order_4.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/poly_order_4/B_fcn_order_4.m','w');
fprintf(fid,'%s',f);
fclose(fid);

%%
% Centrifugal / Coriolis matrix
tic

q_len = length(q);
C_fcn = sym(zeros(q_len));
for i = 1:q_len
    for j = 1:q_len
        for k = 1:q_len
            Christoffel = 0.5*(diff(B_fcn(i,j),q(k)) + diff(B_fcn(i,k),q(j)) - diff(B_fcn(j,k),q(i)));
            C_fcn(i,j) = C_fcn(i,j) + Christoffel*dq(k);
        end
    end
end

toc

matlabFunction(C_fcn,'File','automatically_generated/poly_order_4/C_fcn_order_4','Vars',{p, q, dq}); % creating the MatLab function

fid  = fopen('automatically_generated/poly_order_4/C_fcn_order_4.m','r');
f=fread(fid,'*char')';
fclose(fid);
f = strrep(f,'fresnelc','fresnelc_approx');
f = strrep(f,'fresnels','fresnels_approx');
fid  = fopen('automatically_generated/poly_order_4/C_fcn_order_4.m','w');
fprintf(fid,'%s',f);
fclose(fid);

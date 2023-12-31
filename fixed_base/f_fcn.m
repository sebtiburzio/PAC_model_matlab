function ddx = f_fcn(x,dx)
%
% function ddx = f_fcn(x,dx)
%
% This function implements the dynamics leveraging the other functions
%  automatically generated by model_gen_fixed_base
%

global p_vals G_dir
global K Theta_bar D
global G_Scale

if abs(x(1)) < 1e-5
    x(1) = 1e-5;
end

if abs(x(2)) < 1e-5
    x(2) = 1e-5;
end

B = B_fcn(p_vals,x);
G = Gv_fcn(p_vals,x,G_dir);
C = C_fcn(p_vals,x,dx);

ddx = B\(-C*dx -G*G_Scale -K*(x-Theta_bar) -D*dx + [0; 0]);
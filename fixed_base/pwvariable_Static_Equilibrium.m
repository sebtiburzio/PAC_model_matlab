function E = pwvariable_Static_Equilibrium(z,p_vals,Gamma_here,H,k_obj,theta_bar);


E = Gv_fcn(p_vals,z,Gamma_here)+k_obj*H*(z-theta_bar);

% a=1
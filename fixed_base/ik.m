function [theta, converged] = ik(p_vals, theta_guess, fk_target, epsilon, max_iterations)

    error_2norm_last = Inf;
    for i = 1:max_iterations
        midpt = fk_fcn(p_vals, theta_guess, 0.5, 0);
        endpt = fk_fcn(p_vals, theta_guess, 1, 0);
        error = ([midpt; endpt] - fk_target);
        error_2norm = norm(error);
        if error_2norm < epsilon
                disp("Converged after " + num2str(i) + " iterations")
                theta = theta_guess;
                converged = true;
                return
        else
            if isapprox(error_2norm, error_2norm_last)
                disp("Error stable after iteration " + num2str(i))
                theta = theta_guess;
                converged = false;
                return
            elseif error_2norm > error_2norm_last
                disp("Error increasing after iteration " + num2str(i))
                theta = theta_guess_last;
                converged = false;
                return
            else
                theta_guess_last = theta_guess;
                error_2norm_last = error_2norm;
                J = [J_fcn(p_vals, theta_guess, 0.5, 0); 
                     J_fcn(p_vals, theta_guess, 1, 0)];
                theta_guess = theta_guess - (pinv(J)*error);
            end
        end
    end
    disp("Max iterations reached (check why)")
    theta = theta_guess;
    converged = false;
end
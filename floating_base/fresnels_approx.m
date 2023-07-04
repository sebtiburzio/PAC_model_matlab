function Y = fresnels_approx(X)
%
% This function approximates FRESNELS, which - at least in the version of
%  matlab I am currently using - is implemented through a callback to the
%  symbolic engine
%
% FRESNELS Fresnel sine integral.
%    Y = FRESNELS(X) is the Fresnel sine integral of X.
%    It is defined as:
%    FRESNELS(x) = integral from 0 to x of sin(pi*t^2 / 2) dt 
%    See also FRESNELC.
%
%
% Note!!! This function works only for real or immaginary inputs, not for
%  complex one
%

%
% trying to use the erf function
%
% http://functions.wolfram.com/GammaBetaErf/FresnelS/introductions/FresnelIntegrals/ShowAll.html
%
a1 = X*sqrt(pi)*(1+1i)/2;
a2 = X*sqrt(pi)*(1-1i)/2;
Y = 0.25*(1+1i)*(erf_d(a1) - 1i*erf_d(a2));



%%%%%%%%%%%%%%%%%%%%%%%%%
%
% approximate solution by direct integration. Somehow it seems to not work
% for simulations, while instead works quite well when just comparing the
% outputs of the two funcitons. Why this is the case is a mistery for me
%
%%%%%%%%%%%%%%%%%%%%%%%%%
% step = 1e-2;
% Y = zeros(size(X));
% for iX = 1:length(X)
%     if ~isreal(X(iX))
%         if ~isreal(1i*X(iX))
%             disp('complex number!')
%             X(iX)
%         end
%     end
%     
%     if X(iX) > 5
%         Y(iX) = 0.5;
%     elseif -1i*X(iX) > 5
%         Y(iX) = -1i*0.5;
%     elseif X(iX) < -5
%         Y(iX) = -0.5;
%     elseif -1i*X(iX) < -5
%         Y(iX) =  1i*0.5;
% %     elseif abs(X(iX)) < 2*step
% %         Y(iX) = X(iX)*sin(pi*0^2/2);
%     else
%         t_vect = 0:step:abs(X(iX));
%         Y(iX) = 0;
%         for i_t = 1:length(t_vect)
%             Y(iX) = Y(iX) + step*sin(pi*(t_vect(i_t))^2/2);
%         end
%         if isreal(X(iX))
%             Y(iX) =  sign(X(iX))*Y(iX);
%         else
%             Y(iX) = -sign(X(iX))*Y(iX);
%         end
%     end
% end

end
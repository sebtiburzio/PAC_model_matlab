function fk_fcn = fk_fcn(in1,in2,s,d)
%FK_FCN
%    FK_FCN = FK_FCN(IN1,IN2,S,D)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    17-Aug-2023 12:34:14

D = in1(4,:);
L = in1(3,:);
theta_0 = in2(1,:);
theta_1 = in2(2,:);
t2 = s.*theta_0;
t3 = s.^2;
t4 = theta_0.^2;
t5 = 1.0./theta_1;
t6 = sqrt(pi);
t7 = theta_0./2.0;
t8 = (s.*theta_1)./2.0;
t9 = 1.0./t6;
t10 = sqrt(t5);
t11 = (t3.*theta_1)./2.0;
t12 = (t4.*t5)./2.0;
t15 = t7+t8;
t13 = t2+t11;
t14 = cos(t12);
t16 = sin(t12);
t17 = t9.*t10.*theta_0;
t20 = t9.*t10.*t15.*2.0;
t18 = fresnelc_approx(t17);
t19 = fresnels_approx(t17);
t21 = fresnelc_approx(t20);
t22 = fresnels_approx(t20);
fk_fcn = [L.*(t6.*t10.*t14.*t19-t6.*t10.*t16.*t18-t6.*t10.*t14.*t22+t6.*t10.*t16.*t21)+D.*d.*cos(t13);L.*(t6.*t10.*t14.*t18-t6.*t10.*t14.*t21+t6.*t10.*t16.*t19-t6.*t10.*t16.*t22)-D.*d.*sin(t13)];

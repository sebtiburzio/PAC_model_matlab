function G_fcn = G_fcn(in1,in2)
%G_fcn
%    G_fcn = G_fcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    27-Jan-2024 18:08:42

L = in1(3,:);
m_E = in1(2,:);
m_L = in1(1,:);
phi = in2(5,:);
theta_0 = in2(1,:);
theta_1 = in2(2,:);
t2 = theta_0+theta_1;
t7 = theta_0.^2;
t10 = 1.0./theta_1;
t15 = sqrt(pi);
t11 = t10.^2;
t12 = t10.^3;
t13 = t2+theta_0.*3.0;
t14 = t2.^2;
t17 = 1.0./t15;
t18 = t2.*3.0+theta_0;
t23 = sqrt(t10);
t31 = (t7.*t10)./2.0;
t16 = t13+theta_0.*8.0;
t19 = t13.^2;
t20 = t13.*2.0+t18;
t21 = t13+t18.*2.0;
t24 = t23.^3;
t25 = t2.*8.0+t18;
t26 = t18.^2;
t27 = conj(t23);
t33 = -t31;
t34 = cos(t31);
t35 = sin(t31);
t38 = (t10.*t14)./2.0;
t22 = t16.^2;
t28 = conj(t24);
t29 = t20.^2;
t30 = t21.^2;
t32 = t25.^2;
t36 = 1.0./t27;
t39 = cos(t38);
t40 = sin(t38);
t41 = phi+t33;
t44 = (t10.*t19)./3.2e+1;
t45 = t17.*t27.*theta_0;
t51 = (t10.*t26)./3.2e+1;
t37 = 1.0./t28;
t42 = cos(t41);
t43 = sin(t41);
t46 = cos(t44);
t47 = sin(t44);
t48 = (t10.*t22)./2.88e+2;
t49 = fresnelc_approx(t45);
t50 = fresnels_approx(t45);
t52 = sin(t51);
t55 = cos(t51);
t56 = (t10.*t29)./2.88e+2;
t57 = (t10.*t30)./2.88e+2;
t60 = (t10.*t32)./2.88e+2;
t65 = t10.*t17.*t36;
t67 = t2.*t11.*t17.*t36;
t79 = (t11.*t13.*t17.*t36)./4.0;
t87 = (t11.*t16.*t17.*t36)./1.2e+1;
t90 = (t11.*t17.*t18.*t36)./4.0;
t100 = (t11.*t17.*t20.*t36)./1.2e+1;
t102 = (t11.*t17.*t21.*t36)./1.2e+1;
t112 = (t11.*t17.*t25.*t36)./1.2e+1;
t53 = cos(t48);
t54 = sin(t48);
t58 = sin(t56);
t59 = sin(t57);
t61 = cos(t56);
t62 = cos(t57);
t63 = sin(t60);
t64 = cos(t60);
t66 = t2.*t65;
t68 = t65./4.0;
t69 = t65.*(3.0./4.0);
t72 = t65./1.2e+1;
t73 = t65.*(5.0./1.2e+1);
t74 = t65.*(7.0./1.2e+1);
t75 = -t67;
t76 = t65.*(1.1e+1./1.2e+1);
t77 = (t2.*t12.*t17.*t37)./2.0;
t80 = -t79;
t81 = (t12.*t13.*t17.*t37)./8.0;
t84 = L.*t10.*t34.*t42;
t85 = L.*t10.*t35.*t43;
t88 = (t12.*t16.*t17.*t37)./2.4e+1;
t93 = -t87;
t94 = -t90;
t95 = (t12.*t17.*t18.*t37)./8.0;
t103 = (t12.*t17.*t20.*t37)./2.4e+1;
t104 = (t12.*t17.*t21.*t37)./2.4e+1;
t109 = -t100;
t110 = -t102;
t113 = (t12.*t17.*t25.*t37)./2.4e+1;
t116 = (L.*t11.*t34.*t42.*theta_0)./2.0;
t117 = (L.*t11.*t35.*t43.*theta_0)./2.0;
t118 = -t112;
t120 = L.*t15.*t27.*t42.*t50;
t121 = L.*t15.*t27.*t43.*t49;
t128 = (L.*t11.*t15.*t36.*t42.*t49)./2.0;
t129 = (L.*t11.*t15.*t36.*t43.*t50)./2.0;
t70 = fresnelc_approx(t66);
t71 = fresnels_approx(t66);
t78 = t13.*t68;
t86 = t16.*t72;
t89 = t18.*t68;
t98 = -t85;
t99 = t20.*t72;
t101 = t21.*t72;
t111 = t25.*t72;
t119 = -t117;
t124 = t10.*t121.*theta_0;
t125 = t10.*t120.*theta_0;
t126 = (t7.*t11.*t120)./2.0;
t127 = (t7.*t11.*t121)./2.0;
t130 = -t129;
t131 = t65+t75+t77;
t132 = t68+t80+t81;
t133 = t72+t88+t93;
t134 = t69+t94+t95;
t135 = t73+t103+t109;
t136 = t74+t104+t110;
t137 = t76+t113+t118;
t82 = fresnelc_approx(t78);
t83 = fresnels_approx(t78);
t91 = fresnelc_approx(t89);
t92 = fresnels_approx(t89);
t96 = fresnelc_approx(t86);
t97 = fresnels_approx(t86);
t105 = fresnelc_approx(t99);
t106 = fresnelc_approx(t101);
t107 = fresnels_approx(t99);
t108 = fresnels_approx(t101);
t114 = fresnelc_approx(t111);
t115 = fresnels_approx(t111);
et1 = m_E.*(t84+t98+t124+t125-L.*t10.*t39.*t42+L.*t10.*t40.*t43-L.*t10.*t15.*t27.*t42.*t71.*theta_0-L.*t10.*t15.*t27.*t43.*t70.*theta_0).*(9.81e+2./1.0e+2)+m_L.*(t84+t98+t124+t125-L.*t10.*t42.*t46+L.*t10.*t43.*t47-L.*t10.*t15.*t27.*t42.*t83.*theta_0-L.*t10.*t15.*t27.*t43.*t82.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t84+t98+t124+t125+L.*t10.*t43.*t52-L.*t10.*t42.*t55-L.*t10.*t15.*t27.*t42.*t92.*theta_0-L.*t10.*t15.*t27.*t43.*t91.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t84+t98+t124+t125-L.*t10.*t42.*t53+L.*t10.*t43.*t54-L.*t10.*t15.*t27.*t42.*t97.*theta_0-L.*t10.*t15.*t27.*t43.*t96.*theta_0).*(3.27e+2./2.0e+2);
et2 = m_L.*(t84+t98+t124+t125+L.*t10.*t43.*t58-L.*t10.*t42.*t61-L.*t10.*t15.*t27.*t43.*t105.*theta_0-L.*t10.*t15.*t27.*t42.*t107.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t84+t98+t124+t125+L.*t10.*t43.*t59-L.*t10.*t42.*t62-L.*t10.*t15.*t27.*t43.*t106.*theta_0-L.*t10.*t15.*t27.*t42.*t108.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t84+t98+t124+t125-L.*t10.*t42.*t64+L.*t10.*t43.*t63-L.*t10.*t15.*t27.*t42.*t115.*theta_0-L.*t10.*t15.*t27.*t43.*t114.*theta_0).*(3.27e+2./2.0e+2);
et3 = m_E.*(t116+t119+t126+t127+t128+t130-(L.*t11.*t15.*t36.*t42.*t70)./2.0+(L.*t11.*t15.*t36.*t43.*t71)./2.0+L.*t15.*t27.*t39.*t42.*t131-L.*t15.*t27.*t40.*t43.*t131-(L.*t7.*t11.*t15.*t27.*t42.*t71)./2.0-(L.*t7.*t11.*t15.*t27.*t43.*t70)./2.0).*(-9.81e+2./1.0e+2)-m_L.*(t116+t119+t126+t127+t128+t130-(L.*t11.*t15.*t36.*t42.*t82)./2.0+(L.*t11.*t15.*t36.*t43.*t83)./2.0+L.*t15.*t27.*t42.*t46.*t132-L.*t15.*t27.*t43.*t47.*t132-(L.*t7.*t11.*t15.*t27.*t42.*t83)./2.0-(L.*t7.*t11.*t15.*t27.*t43.*t82)./2.0).*(3.27e+2./2.0e+2);
et4 = m_L.*(t116+t119+t126+t127+t128+t130-(L.*t11.*t15.*t36.*t42.*t91)./2.0+(L.*t11.*t15.*t36.*t43.*t92)./2.0-L.*t15.*t27.*t43.*t52.*t134+L.*t15.*t27.*t42.*t55.*t134-(L.*t7.*t11.*t15.*t27.*t42.*t92)./2.0-(L.*t7.*t11.*t15.*t27.*t43.*t91)./2.0).*(-3.27e+2./2.0e+2)-m_L.*(t116+t119+t126+t127+t128+t130-(L.*t11.*t15.*t36.*t42.*t96)./2.0+(L.*t11.*t15.*t36.*t43.*t97)./2.0+L.*t15.*t27.*t42.*t53.*t133-L.*t15.*t27.*t43.*t54.*t133-(L.*t7.*t11.*t15.*t27.*t42.*t97)./2.0-(L.*t7.*t11.*t15.*t27.*t43.*t96)./2.0).*(3.27e+2./2.0e+2);
et5 = m_L.*(t116+t119+t126+t127+t128+t130-(L.*t11.*t15.*t36.*t42.*t105)./2.0+(L.*t11.*t15.*t36.*t43.*t107)./2.0-L.*t15.*t27.*t43.*t58.*t135+L.*t15.*t27.*t42.*t61.*t135-(L.*t7.*t11.*t15.*t27.*t43.*t105)./2.0-(L.*t7.*t11.*t15.*t27.*t42.*t107)./2.0).*(-3.27e+2./2.0e+2)-m_L.*(t116+t119+t126+t127+t128+t130-(L.*t11.*t15.*t36.*t42.*t106)./2.0+(L.*t11.*t15.*t36.*t43.*t108)./2.0-L.*t15.*t27.*t43.*t59.*t136+L.*t15.*t27.*t42.*t62.*t136-(L.*t7.*t11.*t15.*t27.*t43.*t106)./2.0-(L.*t7.*t11.*t15.*t27.*t42.*t108)./2.0).*(3.27e+2./2.0e+2);
et6 = m_L.*(t116+t119+t126+t127+t128+t130-(L.*t11.*t15.*t36.*t42.*t114)./2.0+(L.*t11.*t15.*t36.*t43.*t115)./2.0+L.*t15.*t27.*t42.*t64.*t137-L.*t15.*t27.*t43.*t63.*t137-(L.*t7.*t11.*t15.*t27.*t42.*t115)./2.0-(L.*t7.*t11.*t15.*t27.*t43.*t114)./2.0).*(-3.27e+2./2.0e+2);
et7 = m_E.*(t120+t121-L.*t15.*t27.*t42.*t71-L.*t15.*t27.*t43.*t70).*(-9.81e+2./1.0e+2)-m_L.*(t120+t121-L.*t15.*t27.*t42.*t83-L.*t15.*t27.*t43.*t82).*(3.27e+2./2.0e+2)-m_L.*(t120+t121-L.*t15.*t27.*t42.*t92-L.*t15.*t27.*t43.*t91).*(3.27e+2./2.0e+2)-m_L.*(t120+t121-L.*t15.*t27.*t42.*t97-L.*t15.*t27.*t43.*t96).*(3.27e+2./2.0e+2)-m_L.*(t120+t121-L.*t15.*t27.*t43.*t105-L.*t15.*t27.*t42.*t107).*(3.27e+2./2.0e+2)-m_L.*(t120+t121-L.*t15.*t27.*t43.*t106-L.*t15.*t27.*t42.*t108).*(3.27e+2./2.0e+2);
et8 = m_L.*(t120+t121-L.*t15.*t27.*t42.*t115-L.*t15.*t27.*t43.*t114).*(-3.27e+2./2.0e+2);
G_fcn = [et1+et2;et3+et4+et5+et6;0.0;m_E.*(9.81e+2./1.0e+2)+m_L.*(9.81e+2./1.0e+2)+7.848e-1;et7+et8];
end

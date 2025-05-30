function Gv_fcn = Gv_fcn(in1,in2,gamma)
%Gv_fcn
%    Gv_fcn = Gv_fcn(IN1,IN2,GAMMA)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    27-Jan-2024 18:08:43

L = in1(3,:);
m_E = in1(2,:);
m_L = in1(1,:);
phi = in2(5,:);
theta_0 = in2(1,:);
theta_1 = in2(2,:);
t2 = conj(gamma);
t5 = theta_0+theta_1;
t10 = theta_0.^2;
t11 = -phi;
t14 = 1.0./theta_1;
t19 = sqrt(pi);
t3 = cos(t2);
t4 = sin(t2);
t15 = t14.^2;
t16 = t14.^3;
t17 = t5+theta_0.*3.0;
t18 = t5.^2;
t21 = 1.0./t19;
t22 = t5.*3.0+theta_0;
t27 = sqrt(t14);
t35 = (t10.*t14)./2.0;
t20 = t17+theta_0.*8.0;
t23 = t17.^2;
t24 = t17.*2.0+t22;
t25 = t17+t22.*2.0;
t28 = t27.^3;
t29 = t5.*8.0+t22;
t30 = t22.^2;
t31 = conj(t27);
t37 = cos(t35);
t38 = sin(t35);
t41 = (t14.*t18)./2.0;
t48 = t2+t11+t35;
t26 = t20.^2;
t32 = conj(t28);
t33 = t24.^2;
t34 = t25.^2;
t36 = t29.^2;
t39 = 1.0./t31;
t42 = cos(t41);
t43 = sin(t41);
t44 = (t14.*t23)./3.2e+1;
t45 = t21.*t31.*theta_0;
t52 = (t14.*t30)./3.2e+1;
t54 = cos(t48);
t55 = sin(t48);
t40 = 1.0./t32;
t46 = cos(t44);
t47 = sin(t44);
t49 = (t14.*t26)./2.88e+2;
t50 = fresnelc_approx(t45);
t51 = fresnels_approx(t45);
t53 = sin(t52);
t58 = cos(t52);
t59 = (t14.*t33)./2.88e+2;
t60 = (t14.*t34)./2.88e+2;
t63 = (t14.*t36)./2.88e+2;
t68 = t14.*t21.*t39;
t70 = t5.*t15.*t21.*t39;
t82 = (t15.*t17.*t21.*t39)./4.0;
t88 = (t15.*t20.*t21.*t39)./1.2e+1;
t91 = (t15.*t21.*t22.*t39)./4.0;
t100 = (t15.*t21.*t24.*t39)./1.2e+1;
t102 = (t15.*t21.*t25.*t39)./1.2e+1;
t112 = (t15.*t21.*t29.*t39)./1.2e+1;
t116 = L.*t14.*t37.*t54;
t117 = L.*t14.*t38.*t55;
t121 = (L.*t15.*t37.*t54.*theta_0)./2.0;
t122 = (L.*t15.*t38.*t55.*theta_0)./2.0;
t56 = cos(t49);
t57 = sin(t49);
t61 = sin(t59);
t62 = sin(t60);
t64 = cos(t59);
t65 = cos(t60);
t66 = sin(t63);
t67 = cos(t63);
t69 = t5.*t68;
t71 = t68./4.0;
t72 = t68.*(3.0./4.0);
t75 = t68./1.2e+1;
t76 = t68.*(5.0./1.2e+1);
t77 = t68.*(7.0./1.2e+1);
t78 = -t70;
t79 = t68.*(1.1e+1./1.2e+1);
t80 = (t5.*t16.*t21.*t40)./2.0;
t83 = -t82;
t84 = (t16.*t17.*t21.*t40)./8.0;
t89 = (t16.*t20.*t21.*t40)./2.4e+1;
t94 = -t88;
t95 = -t91;
t96 = (t16.*t21.*t22.*t40)./8.0;
t103 = (t16.*t21.*t24.*t40)./2.4e+1;
t104 = (t16.*t21.*t25.*t40)./2.4e+1;
t109 = -t100;
t110 = -t102;
t113 = (t16.*t21.*t29.*t40)./2.4e+1;
t118 = -t112;
t123 = L.*t19.*t31.*t51.*t54;
t124 = L.*t19.*t31.*t50.*t55;
t130 = (L.*t15.*t19.*t39.*t50.*t54)./2.0;
t131 = (L.*t15.*t19.*t39.*t51.*t55)./2.0;
t73 = fresnelc_approx(t69);
t74 = fresnels_approx(t69);
t81 = t17.*t71;
t87 = t20.*t75;
t90 = t22.*t71;
t99 = t24.*t75;
t101 = t25.*t75;
t111 = t29.*t75;
t126 = t14.*t123.*theta_0;
t127 = t14.*t124.*theta_0;
t132 = (t10.*t15.*t123)./2.0;
t133 = (t10.*t15.*t124)./2.0;
t135 = t68+t78+t80;
t136 = t71+t83+t84;
t137 = t75+t89+t94;
t138 = t72+t95+t96;
t139 = t76+t103+t109;
t140 = t77+t104+t110;
t141 = t79+t113+t118;
t85 = fresnelc_approx(t81);
t86 = fresnels_approx(t81);
t92 = fresnelc_approx(t90);
t93 = fresnels_approx(t90);
t97 = fresnelc_approx(t87);
t98 = fresnels_approx(t87);
t105 = fresnelc_approx(t99);
t106 = fresnelc_approx(t101);
t107 = fresnels_approx(t99);
t108 = fresnels_approx(t101);
t114 = fresnelc_approx(t111);
t115 = fresnels_approx(t111);
t129 = -t127;
t134 = -t133;
et1 = m_E.*(t116+t117+t126+t129-L.*t14.*t42.*t54-L.*t14.*t43.*t55-L.*t14.*t19.*t31.*t54.*t74.*theta_0+L.*t14.*t19.*t31.*t55.*t73.*theta_0).*(9.81e+2./1.0e+2)+m_L.*(t116+t117+t126+t129-L.*t14.*t46.*t54-L.*t14.*t47.*t55-L.*t14.*t19.*t31.*t54.*t86.*theta_0+L.*t14.*t19.*t31.*t55.*t85.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t116+t117+t126+t129-L.*t14.*t53.*t55-L.*t14.*t54.*t58-L.*t14.*t19.*t31.*t54.*t93.*theta_0+L.*t14.*t19.*t31.*t55.*t92.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t116+t117+t126+t129-L.*t14.*t54.*t56-L.*t14.*t55.*t57-L.*t14.*t19.*t31.*t54.*t98.*theta_0+L.*t14.*t19.*t31.*t55.*t97.*theta_0).*(3.27e+2./2.0e+2);
et2 = m_L.*(t116+t117+t126+t129-L.*t14.*t55.*t61-L.*t14.*t54.*t64+L.*t14.*t19.*t31.*t55.*t105.*theta_0-L.*t14.*t19.*t31.*t54.*t107.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t116+t117+t126+t129-L.*t14.*t55.*t62-L.*t14.*t54.*t65+L.*t14.*t19.*t31.*t55.*t106.*theta_0-L.*t14.*t19.*t31.*t54.*t108.*theta_0).*(3.27e+2./2.0e+2)+m_L.*(t116+t117+t126+t129-L.*t14.*t54.*t67-L.*t14.*t55.*t66-L.*t14.*t19.*t31.*t54.*t115.*theta_0+L.*t14.*t19.*t31.*t55.*t114.*theta_0).*(3.27e+2./2.0e+2);
et3 = m_E.*(t121+t122+t130+t131+t132+t134-(L.*t15.*t19.*t39.*t54.*t73)./2.0-(L.*t15.*t19.*t39.*t55.*t74)./2.0+L.*t19.*t31.*t42.*t54.*t135+L.*t19.*t31.*t43.*t55.*t135-(L.*t10.*t15.*t19.*t31.*t54.*t74)./2.0+(L.*t10.*t15.*t19.*t31.*t55.*t73)./2.0).*(-9.81e+2./1.0e+2)-m_L.*(t121+t122+t130+t131+t132+t134-(L.*t15.*t19.*t39.*t54.*t85)./2.0-(L.*t15.*t19.*t39.*t55.*t86)./2.0+L.*t19.*t31.*t46.*t54.*t136+L.*t19.*t31.*t47.*t55.*t136-(L.*t10.*t15.*t19.*t31.*t54.*t86)./2.0+(L.*t10.*t15.*t19.*t31.*t55.*t85)./2.0).*(3.27e+2./2.0e+2);
et4 = m_L.*(t121+t122+t130+t131+t132+t134-(L.*t15.*t19.*t39.*t54.*t92)./2.0-(L.*t15.*t19.*t39.*t55.*t93)./2.0+L.*t19.*t31.*t53.*t55.*t138+L.*t19.*t31.*t54.*t58.*t138-(L.*t10.*t15.*t19.*t31.*t54.*t93)./2.0+(L.*t10.*t15.*t19.*t31.*t55.*t92)./2.0).*(-3.27e+2./2.0e+2)-m_L.*(t121+t122+t130+t131+t132+t134-(L.*t15.*t19.*t39.*t54.*t97)./2.0-(L.*t15.*t19.*t39.*t55.*t98)./2.0+L.*t19.*t31.*t54.*t56.*t137+L.*t19.*t31.*t55.*t57.*t137-(L.*t10.*t15.*t19.*t31.*t54.*t98)./2.0+(L.*t10.*t15.*t19.*t31.*t55.*t97)./2.0).*(3.27e+2./2.0e+2);
et5 = m_L.*(t121+t122+t130+t131+t132+t134-(L.*t15.*t19.*t39.*t54.*t105)./2.0-(L.*t15.*t19.*t39.*t55.*t107)./2.0+L.*t19.*t31.*t55.*t61.*t139+L.*t19.*t31.*t54.*t64.*t139+(L.*t10.*t15.*t19.*t31.*t55.*t105)./2.0-(L.*t10.*t15.*t19.*t31.*t54.*t107)./2.0).*(-3.27e+2./2.0e+2)-m_L.*(t121+t122+t130+t131+t132+t134-(L.*t15.*t19.*t39.*t54.*t106)./2.0-(L.*t15.*t19.*t39.*t55.*t108)./2.0+L.*t19.*t31.*t55.*t62.*t140+L.*t19.*t31.*t54.*t65.*t140+(L.*t10.*t15.*t19.*t31.*t55.*t106)./2.0-(L.*t10.*t15.*t19.*t31.*t54.*t108)./2.0).*(3.27e+2./2.0e+2);
et6 = m_L.*(t121+t122+t130+t131+t132+t134-(L.*t15.*t19.*t39.*t54.*t114)./2.0-(L.*t15.*t19.*t39.*t55.*t115)./2.0+L.*t19.*t31.*t54.*t67.*t141+L.*t19.*t31.*t55.*t66.*t141-(L.*t10.*t15.*t19.*t31.*t54.*t115)./2.0+(L.*t10.*t15.*t19.*t31.*t55.*t114)./2.0).*(-3.27e+2./2.0e+2);
et7 = m_E.*(t123-t124-L.*t19.*t31.*t54.*t74+L.*t19.*t31.*t55.*t73).*(-9.81e+2./1.0e+2)-m_L.*(t123-t124-L.*t19.*t31.*t54.*t86+L.*t19.*t31.*t55.*t85).*(3.27e+2./2.0e+2)-m_L.*(t123-t124-L.*t19.*t31.*t54.*t93+L.*t19.*t31.*t55.*t92).*(3.27e+2./2.0e+2)-m_L.*(t123-t124-L.*t19.*t31.*t54.*t98+L.*t19.*t31.*t55.*t97).*(3.27e+2./2.0e+2)-m_L.*(t123-t124+L.*t19.*t31.*t55.*t105-L.*t19.*t31.*t54.*t107).*(3.27e+2./2.0e+2)-m_L.*(t123-t124+L.*t19.*t31.*t55.*t106-L.*t19.*t31.*t54.*t108).*(3.27e+2./2.0e+2);
et8 = m_L.*(t123-t124-L.*t19.*t31.*t54.*t115+L.*t19.*t31.*t55.*t114).*(-3.27e+2./2.0e+2);
Gv_fcn = [et1+et2;et3+et4+et5+et6;t4.*7.848e-1+m_E.*t4.*(9.81e+2./1.0e+2)+m_L.*t4.*(9.81e+2./1.0e+2);t3.*7.848e-1+m_E.*t3.*(9.81e+2./1.0e+2)+m_L.*t3.*(9.81e+2./1.0e+2);et7+et8];
end

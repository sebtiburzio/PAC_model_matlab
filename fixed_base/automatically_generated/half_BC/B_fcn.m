function B_fcn = B_fcn(in1,in2)
%B_fcn
%    B_fcn = B_fcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    17-Aug-2023 12:34:47

D = in1(4,:);
L = in1(3,:);
m_E = in1(2,:);
m_L = in1(1,:);
theta_0 = in2(1,:);
theta_1 = in2(2,:);
t2 = conj(theta_0);
t3 = conj(theta_1);
t4 = D.^2;
t5 = L.^2;
t6 = theta_0.^2;
t8 = 1.0./theta_1;
t13 = sqrt(pi);
t14 = theta_0./2.0;
t15 = theta_1./2.0;
t16 = theta_0./4.0;
t17 = theta_0.*(3.0./4.0);
t23 = theta_1./8.0;
t24 = theta_1.*(3.0./8.0);
t25 = theta_0./1.2e+1;
t26 = theta_0.*(5.0./1.2e+1);
t27 = theta_0.*(7.0./1.2e+1);
t28 = theta_1./2.4e+1;
t29 = theta_1.*(5.0./2.4e+1);
t30 = theta_1.*(7.0./2.4e+1);
t31 = theta_1./3.2e+1;
t45 = theta_0.*(1.1e+1./1.2e+1);
t46 = theta_1.*(1.1e+1./2.4e+1);
t47 = theta_1.*(9.0./3.2e+1);
t48 = theta_1./2.88e+2;
t58 = theta_1.*(2.5e+1./2.88e+2);
t59 = theta_1.*(4.9e+1./2.88e+2);
t68 = theta_1.*(1.21e+2./2.88e+2);
t7 = t2.^2;
t9 = t8.^2;
t10 = -t2;
t11 = 1.0./t3;
t18 = t2./2.0;
t19 = t3./2.0;
t20 = t2./4.0;
t21 = t2.*(3.0./4.0);
t22 = 1.0./t13;
t32 = -t8;
t36 = t3./8.0;
t37 = t3.*(3.0./8.0);
t38 = t2./1.2e+1;
t39 = t2.*(5.0./1.2e+1);
t40 = t2.*(7.0./1.2e+1);
t41 = t3./2.4e+1;
t42 = t3.*(5.0./2.4e+1);
t43 = t3.*(7.0./2.4e+1);
t44 = t3./3.2e+1;
t49 = sqrt(t8);
t53 = t2.*(1.1e+1./1.2e+1);
t55 = t3.*(1.1e+1./2.4e+1);
t56 = t3.*(9.0./3.2e+1);
t57 = t3./2.88e+2;
t64 = t3.*(2.5e+1./2.88e+2);
t65 = t3.*(4.9e+1./2.88e+2);
t71 = t3.*(1.21e+2./2.88e+2);
t72 = t14+t15;
t73 = (t6.*t8)./2.0;
t76 = t14+t23;
t77 = t14+t24;
t78 = t14+t28;
t79 = t14+t29;
t80 = t14+t30;
t84 = t14+t46;
t12 = t11.^2;
t33 = -t19;
t34 = -t20;
t35 = -t21;
t50 = -t38;
t51 = -t39;
t52 = -t40;
t54 = -t44;
t60 = 1.0./t49;
t61 = -t53;
t62 = -t56;
t63 = -t57;
t66 = conj(t49);
t67 = t9.*t14;
t69 = -t64;
t70 = -t65;
t74 = -t71;
t81 = cos(t73);
t82 = sin(t73);
t85 = t72.^2;
t86 = t18+t19;
t87 = (t7.*t11)./2.0;
t90 = t76.^2;
t91 = t77.^2;
t92 = t78.^2;
t93 = t79.^2;
t94 = t80.^2;
t95 = t18+t36;
t96 = t18+t37;
t97 = t18+t41;
t98 = t18+t42;
t99 = t18+t43;
t102 = t22.*t49;
t103 = t84.^2;
t104 = t18+t55;
t75 = t12.*t18;
t83 = 1.0./t66;
t88 = cos(t87);
t89 = sin(t87);
t100 = -t87;
t101 = t86.^2;
t105 = t95.^2;
t106 = t96.^2;
t107 = t97.^2;
t108 = t98.^2;
t109 = t99.^2;
t110 = t22.*t66;
t111 = t102.*theta_0;
t114 = t104.^2;
t115 = t102./4.0;
t116 = t102.*(3.0./4.0);
t118 = t10+t15+t33+theta_0;
t119 = t102./1.2e+1;
t120 = t102.*(5.0./1.2e+1);
t121 = t102.*(7.0./1.2e+1);
t122 = t8.*t85.*2.0;
t127 = t102.*(1.1e+1./1.2e+1);
t130 = t8.*t90.*2.0;
t131 = t8.*t91.*2.0;
t132 = t8.*t92.*2.0;
t133 = t8.*t93.*2.0;
t134 = t8.*t94.*2.0;
t143 = t8.*t103.*2.0;
t153 = t72.*t102.*2.0;
t154 = t16+t31+t34+t54;
t155 = t76.*t102.*2.0;
t156 = t77.*t102.*2.0;
t157 = t78.*t102.*2.0;
t158 = t79.*t102.*2.0;
t159 = t80.*t102.*2.0;
t164 = t84.*t102.*2.0;
t165 = t17+t35+t47+t62;
t180 = t22.*1.0./t60.^3.*t72;
t186 = t25+t48+t50+t63;
t190 = t22.*1.0./t60.^3.*t76;
t191 = t22.*1.0./t60.^3.*t77;
t192 = t22.*1.0./t60.^3.*t78;
t193 = t22.*1.0./t60.^3.*t79;
t194 = t22.*1.0./t60.^3.*t80;
t213 = t22.*1.0./t60.^3.*t84;
t216 = t26+t51+t58+t69;
t217 = t27+t52+t59+t70;
t242 = t45+t61+t68+t74;
t112 = fresnelc_approx(t111);
t113 = fresnels_approx(t111);
t117 = t2.*t110;
t123 = t110./4.0;
t124 = t110.*(3.0./4.0);
t128 = cos(t118);
t129 = -t122;
t135 = t110./1.2e+1;
t136 = t110.*(5.0./1.2e+1);
t137 = t110.*(7.0./1.2e+1);
t138 = -t130;
t139 = -t131;
t140 = -t132;
t141 = -t133;
t142 = -t134;
t144 = t110.*(1.1e+1./1.2e+1);
t145 = -t143;
t146 = t11.*t101.*2.0;
t147 = t11.*t105.*2.0;
t148 = t11.*t106.*2.0;
t149 = t11.*t107.*2.0;
t150 = t11.*t108.*2.0;
t151 = t11.*t109.*2.0;
t152 = t11.*t114.*2.0;
t161 = fresnelc_approx(t153);
t162 = fresnels_approx(t153);
t163 = cos(t154);
t166 = fresnelc_approx(t155);
t167 = fresnelc_approx(t156);
t168 = fresnelc_approx(t157);
t169 = fresnels_approx(t155);
t170 = fresnels_approx(t156);
t171 = fresnelc_approx(t158);
t172 = fresnelc_approx(t159);
t173 = fresnels_approx(t157);
t174 = fresnels_approx(t158);
t175 = fresnels_approx(t159);
t176 = t86.*t110.*2.0;
t177 = fresnelc_approx(t164);
t178 = fresnels_approx(t164);
t179 = cos(t165);
t181 = t95.*t110.*2.0;
t182 = t96.*t110.*2.0;
t183 = t97.*t110.*2.0;
t184 = t98.*t110.*2.0;
t185 = t99.*t110.*2.0;
t195 = t104.*t110.*2.0;
t196 = cos(t186);
t212 = -t180;
t221 = -t190;
t222 = -t191;
t223 = -t192;
t224 = -t193;
t225 = -t194;
t236 = cos(t216);
t237 = cos(t217);
t238 = -t213;
t241 = t12.*t22.*t83.*t86;
t244 = t12.*t22.*t83.*t95;
t245 = t12.*t22.*t83.*t96;
t246 = t12.*t22.*t83.*t97;
t247 = t12.*t22.*t83.*t98;
t248 = t12.*t22.*t83.*t99;
t253 = cos(t242);
t261 = t12.*t22.*t83.*t104;
t125 = fresnelc_approx(t117);
t126 = fresnels_approx(t117);
t160 = (t4.*t128)./2.4e+1;
t187 = fresnelc_approx(t176);
t188 = fresnels_approx(t176);
t189 = t73+t129;
t197 = fresnelc_approx(t181);
t198 = fresnelc_approx(t182);
t199 = fresnelc_approx(t183);
t200 = fresnels_approx(t181);
t201 = fresnels_approx(t182);
t202 = fresnelc_approx(t184);
t203 = fresnelc_approx(t185);
t204 = fresnels_approx(t183);
t205 = fresnels_approx(t184);
t206 = fresnels_approx(t185);
t207 = t73+t138;
t208 = t73+t139;
t209 = t73+t140;
t210 = t73+t141;
t211 = t73+t142;
t218 = fresnelc_approx(t195);
t219 = fresnels_approx(t195);
t220 = t73+t145;
t243 = t100+t146;
t254 = (t4.*t163)./1.536e+3;
t255 = t100+t147;
t256 = t100+t148;
t257 = t100+t149;
t258 = t100+t150;
t259 = t100+t151;
t260 = -t241;
t282 = t100+t152;
t283 = -t244;
t284 = -t245;
t285 = -t246;
t286 = -t247;
t287 = -t248;
t292 = -t261;
t293 = t4.*t179.*(9.0./5.12e+2);
t294 = (t4.*t196)./4.1472e+4;
t310 = t4.*t236.*3.014081790123457e-3;
t311 = t4.*t237.*8.270640432098765e-3;
t317 = t13.*1.0./t60.^3.*t81.*t112.*theta_0;
t318 = t13.*1.0./t60.^3.*t81.*t113.*theta_0;
t319 = t13.*1.0./t60.^3.*t82.*t112.*theta_0;
t320 = t13.*1.0./t60.^3.*t82.*t113.*theta_0;
t322 = t102+t212;
t326 = (t13.*1.0./t60.^3.*t81.*t112)./2.0;
t327 = (t13.*1.0./t60.^3.*t81.*t113)./2.0;
t328 = (t13.*1.0./t60.^3.*t82.*t112)./2.0;
t329 = (t13.*1.0./t60.^3.*t82.*t113)./2.0;
t331 = t4.*t253.*3.209394290123457e-2;
t336 = t115+t221;
t337 = t116+t222;
t338 = t6.*t13.*1.0./t60.^5.*t82.*t112.*(-1.0./2.0);
t339 = t119+t223;
t340 = t120+t224;
t341 = t121+t225;
t347 = t127+t238;
t353 = t13.*1.0./t60.^3.*t81.*t161.*theta_0;
t354 = t13.*1.0./t60.^3.*t81.*t162.*theta_0;
t355 = t13.*1.0./t60.^3.*t82.*t161.*theta_0;
t356 = t13.*1.0./t60.^3.*t82.*t162.*theta_0;
t364 = t13.*1.0./t60.^3.*t81.*t166.*theta_0;
t365 = t13.*1.0./t60.^3.*t81.*t167.*theta_0;
t366 = t13.*1.0./t60.^3.*t81.*t168.*theta_0;
t367 = t13.*1.0./t60.^3.*t81.*t169.*theta_0;
t368 = t13.*1.0./t60.^3.*t81.*t170.*theta_0;
t369 = t13.*1.0./t60.^3.*t81.*t171.*theta_0;
t370 = t13.*1.0./t60.^3.*t81.*t172.*theta_0;
t371 = t13.*1.0./t60.^3.*t81.*t173.*theta_0;
t372 = t13.*1.0./t60.^3.*t81.*t174.*theta_0;
t373 = t13.*1.0./t60.^3.*t81.*t175.*theta_0;
t374 = t13.*1.0./t60.^3.*t82.*t166.*theta_0;
t375 = t13.*1.0./t60.^3.*t82.*t167.*theta_0;
t376 = t13.*1.0./t60.^3.*t82.*t168.*theta_0;
t377 = t13.*1.0./t60.^3.*t82.*t169.*theta_0;
t378 = t13.*1.0./t60.^3.*t82.*t170.*theta_0;
t379 = t13.*1.0./t60.^3.*t82.*t171.*theta_0;
t380 = t13.*1.0./t60.^3.*t82.*t172.*theta_0;
t381 = t13.*1.0./t60.^3.*t82.*t173.*theta_0;
t382 = t13.*1.0./t60.^3.*t82.*t174.*theta_0;
t383 = t13.*1.0./t60.^3.*t82.*t175.*theta_0;
t390 = t13.*1.0./t60.^3.*t81.*t177.*theta_0;
t391 = t13.*1.0./t60.^3.*t81.*t178.*theta_0;
t393 = t13.*1.0./t60.^3.*t82.*t177.*theta_0;
t394 = t13.*1.0./t60.^3.*t82.*t178.*theta_0;
t395 = (t13.*1.0./t60.^3.*t81.*t161)./2.0;
t396 = (t13.*1.0./t60.^3.*t81.*t162)./2.0;
t397 = (t13.*1.0./t60.^3.*t82.*t161)./2.0;
t398 = (t13.*1.0./t60.^3.*t82.*t162)./2.0;
t406 = (t13.*1.0./t60.^3.*t81.*t166)./2.0;
t407 = (t13.*1.0./t60.^3.*t81.*t167)./2.0;
t408 = (t13.*1.0./t60.^3.*t81.*t168)./2.0;
t409 = (t13.*1.0./t60.^3.*t81.*t169)./2.0;
t410 = (t13.*1.0./t60.^3.*t81.*t170)./2.0;
t411 = (t13.*1.0./t60.^3.*t81.*t171)./2.0;
t412 = (t13.*1.0./t60.^3.*t81.*t172)./2.0;
t413 = (t13.*1.0./t60.^3.*t81.*t173)./2.0;
t414 = (t13.*1.0./t60.^3.*t81.*t174)./2.0;
t415 = (t13.*1.0./t60.^3.*t81.*t175)./2.0;
t417 = (t13.*1.0./t60.^3.*t82.*t166)./2.0;
t418 = (t13.*1.0./t60.^3.*t82.*t167)./2.0;
t420 = (t13.*1.0./t60.^3.*t82.*t168)./2.0;
t421 = (t13.*1.0./t60.^3.*t82.*t169)./2.0;
t422 = (t13.*1.0./t60.^3.*t82.*t170)./2.0;
t423 = (t13.*1.0./t60.^3.*t82.*t171)./2.0;
t424 = (t13.*1.0./t60.^3.*t82.*t172)./2.0;
t425 = (t13.*1.0./t60.^3.*t82.*t173)./2.0;
t426 = (t13.*1.0./t60.^3.*t82.*t174)./2.0;
t427 = (t13.*1.0./t60.^3.*t82.*t175)./2.0;
t438 = (t13.*1.0./t60.^3.*t81.*t177)./2.0;
t439 = (t13.*1.0./t60.^3.*t81.*t178)./2.0;
t447 = (t13.*1.0./t60.^3.*t82.*t177)./2.0;
t451 = (t13.*1.0./t60.^3.*t82.*t178)./2.0;
t452 = t6.*t13.*1.0./t60.^5.*t81.*t161.*(-1.0./2.0);
t455 = t6.*t13.*1.0./t60.^5.*t81.*t162.*(-1.0./2.0);
t466 = t6.*t13.*1.0./t60.^5.*t82.*t162.*(-1.0./2.0);
t478 = t6.*t13.*1.0./t60.^5.*t81.*t166.*(-1.0./2.0);
t479 = t6.*t13.*1.0./t60.^5.*t81.*t167.*(-1.0./2.0);
t480 = t6.*t13.*1.0./t60.^5.*t81.*t168.*(-1.0./2.0);
t481 = t6.*t13.*1.0./t60.^5.*t81.*t169.*(-1.0./2.0);
t482 = t6.*t13.*1.0./t60.^5.*t81.*t170.*(-1.0./2.0);
t483 = t6.*t13.*1.0./t60.^5.*t81.*t171.*(-1.0./2.0);
t484 = t6.*t13.*1.0./t60.^5.*t81.*t172.*(-1.0./2.0);
t486 = t6.*t13.*1.0./t60.^5.*t81.*t173.*(-1.0./2.0);
t487 = t6.*t13.*1.0./t60.^5.*t81.*t174.*(-1.0./2.0);
t488 = t6.*t13.*1.0./t60.^5.*t81.*t175.*(-1.0./2.0);
t490 = t6.*t13.*1.0./t60.^5.*t82.*t169.*(-1.0./2.0);
t491 = t6.*t13.*1.0./t60.^5.*t82.*t170.*(-1.0./2.0);
t493 = t6.*t13.*1.0./t60.^5.*t82.*t173.*(-1.0./2.0);
t494 = t6.*t13.*1.0./t60.^5.*t82.*t174.*(-1.0./2.0);
t495 = t6.*t13.*1.0./t60.^5.*t82.*t175.*(-1.0./2.0);
t497 = t6.*t13.*1.0./t60.^5.*t81.*t177.*(-1.0./2.0);
t498 = t6.*t13.*1.0./t60.^5.*t81.*t178.*(-1.0./2.0);
t499 = t6.*t13.*1.0./t60.^5.*t82.*t178.*(-1.0./2.0);
t214 = cos(t189);
t215 = sin(t189);
t226 = cos(t207);
t227 = cos(t208);
t228 = cos(t209);
t229 = cos(t210);
t230 = cos(t211);
t231 = sin(t207);
t232 = sin(t208);
t233 = sin(t209);
t234 = sin(t210);
t235 = sin(t211);
t239 = cos(t220);
t240 = sin(t220);
t249 = cos(t243);
t250 = sin(t243);
t262 = cos(t255);
t263 = cos(t256);
t264 = cos(t257);
t265 = cos(t258);
t266 = cos(t259);
t267 = sin(t255);
t268 = sin(t256);
t269 = sin(t257);
t270 = sin(t258);
t271 = sin(t259);
t288 = cos(t282);
t289 = sin(t282);
t323 = -t317;
t324 = -t318;
t325 = -t320;
t330 = -t327;
t332 = t6.*1.0./t60.^2.*t326;
t333 = t6.*1.0./t60.^2.*t327;
t334 = t6.*1.0./t60.^2.*t328;
t335 = t6.*1.0./t60.^2.*t329;
t342 = t2.*t11.*t13.*t66.*t88.*t125;
t343 = t2.*t11.*t13.*t66.*t88.*t126;
t344 = t2.*t11.*t13.*t66.*t89.*t125;
t345 = t2.*t11.*t13.*t66.*t89.*t126;
t346 = t110+t260;
t348 = t10.*t11.*t13.*t66.*t89.*t125;
t349 = (t12.*t13.*t83.*t88.*t125)./2.0;
t350 = (t12.*t13.*t83.*t88.*t126)./2.0;
t351 = (t12.*t13.*t83.*t89.*t125)./2.0;
t352 = (t12.*t13.*t83.*t89.*t126)./2.0;
t357 = t123+t283;
t358 = t124+t284;
t359 = (t7.*t12.*t13.*t66.*t88.*t125)./2.0;
t360 = (t7.*t12.*t13.*t66.*t88.*t126)./2.0;
t361 = (t7.*t12.*t13.*t66.*t89.*t125)./2.0;
t362 = (t7.*t12.*t13.*t66.*t89.*t126)./2.0;
t384 = t135+t285;
t385 = t136+t286;
t386 = t137+t287;
t392 = -t355;
t399 = -t374;
t400 = -t375;
t401 = -t376;
t402 = -t379;
t403 = -t380;
t404 = t144+t292;
t405 = -t395;
t416 = -t397;
t419 = -t398;
t428 = -t393;
t429 = t6.*1.0./t60.^2.*t395;
t430 = t6.*1.0./t60.^2.*t396;
t431 = t6.*1.0./t60.^2.*t397;
t432 = t6.*1.0./t60.^2.*t398;
t433 = -t406;
t434 = -t407;
t435 = -t408;
t436 = -t411;
t437 = -t412;
t440 = -t417;
t441 = -t418;
t442 = -t420;
t443 = -t421;
t444 = -t422;
t445 = -t423;
t446 = -t424;
t448 = -t425;
t449 = -t426;
t450 = -t427;
t453 = t6.*1.0./t60.^2.*t406;
t454 = t6.*1.0./t60.^2.*t407;
t456 = t6.*1.0./t60.^2.*t408;
t457 = t6.*1.0./t60.^2.*t409;
t458 = t6.*1.0./t60.^2.*t410;
t459 = t6.*1.0./t60.^2.*t411;
t460 = t6.*1.0./t60.^2.*t412;
t461 = t6.*1.0./t60.^2.*t413;
t462 = t6.*1.0./t60.^2.*t414;
t463 = t6.*1.0./t60.^2.*t415;
t464 = t6.*1.0./t60.^2.*t417;
t465 = t6.*1.0./t60.^2.*t418;
t467 = t6.*1.0./t60.^2.*t420;
t468 = t6.*1.0./t60.^2.*t421;
t469 = t6.*1.0./t60.^2.*t422;
t470 = t6.*1.0./t60.^2.*t423;
t471 = t6.*1.0./t60.^2.*t424;
t472 = t6.*1.0./t60.^2.*t425;
t473 = t6.*1.0./t60.^2.*t426;
t474 = t6.*1.0./t60.^2.*t427;
t475 = -t438;
t476 = -t447;
t477 = -t451;
t485 = t6.*1.0./t60.^2.*t438;
t489 = t6.*1.0./t60.^2.*t439;
t492 = t6.*1.0./t60.^2.*t447;
t496 = t6.*1.0./t60.^2.*t451;
t500 = t2.*t11.*t13.*t66.*t88.*t187;
t501 = t2.*t11.*t13.*t66.*t88.*t188;
t502 = t2.*t11.*t13.*t66.*t89.*t187;
t503 = t2.*t11.*t13.*t66.*t89.*t188;
t504 = t2.*t11.*t13.*t66.*t88.*t197;
t505 = t2.*t11.*t13.*t66.*t88.*t198;
t506 = t2.*t11.*t13.*t66.*t88.*t199;
t507 = t2.*t11.*t13.*t66.*t88.*t200;
t508 = t2.*t11.*t13.*t66.*t88.*t201;
t509 = t2.*t11.*t13.*t66.*t88.*t202;
t510 = t2.*t11.*t13.*t66.*t88.*t203;
t511 = t2.*t11.*t13.*t66.*t88.*t204;
t512 = t2.*t11.*t13.*t66.*t88.*t205;
t513 = t2.*t11.*t13.*t66.*t88.*t206;
t514 = t2.*t11.*t13.*t66.*t89.*t197;
t515 = t2.*t11.*t13.*t66.*t89.*t198;
t516 = t2.*t11.*t13.*t66.*t89.*t199;
t517 = t2.*t11.*t13.*t66.*t89.*t200;
t518 = t2.*t11.*t13.*t66.*t89.*t201;
t519 = t2.*t11.*t13.*t66.*t89.*t202;
t520 = t2.*t11.*t13.*t66.*t89.*t203;
t521 = t2.*t11.*t13.*t66.*t89.*t204;
t522 = t2.*t11.*t13.*t66.*t89.*t205;
t523 = t2.*t11.*t13.*t66.*t89.*t206;
t524 = t10.*t11.*t13.*t66.*t88.*t187;
t525 = t10.*t11.*t13.*t66.*t88.*t188;
t526 = t2.*t11.*t13.*t66.*t88.*t218;
t527 = t2.*t11.*t13.*t66.*t88.*t219;
t528 = t10.*t11.*t13.*t66.*t89.*t188;
t529 = t2.*t11.*t13.*t66.*t89.*t218;
t530 = t2.*t11.*t13.*t66.*t89.*t219;
t531 = t10.*t11.*t13.*t66.*t88.*t197;
t532 = t10.*t11.*t13.*t66.*t88.*t198;
t533 = t10.*t11.*t13.*t66.*t88.*t199;
t534 = t10.*t11.*t13.*t66.*t88.*t200;
B_fcn = ft_1({m_E,m_L,t10,t11,t12,t126,t128,t13,t160,t163,t179,t187,t188,t196,t197,t198,t199,t200,t201,t202,t203,t204,t205,t206,t214,t215,t218,t219,t226,t227,t228,t229,t230,t231,t232,t233,t234,t235,t236,t237,t239,t240,t249,t250,t253,t254,t262,t263,t264,t265,t266,t267,t268,t269,t270,t271,t288,t289,t293,t294,t310,t311,t318,t319,t32,t322,t323,t324,t325,t326,t328,t329,t330,t331,t332,t333,t335,t336,t337,t338,t339,t340,t341,t342,t343,t344,t345,t346,t347,t348,t349,t350,t351,t352,t353,t354,t355,t356,t357,t358,t359,t360,t361,t362,t364,t365,t366,t367,t368,t369,t370,t371,t372,t373,t374,t375,t376,t377,t378,t379,t380,t381,t382,t383,t384,t385,t386,t390,t391,t392,t393,t394,t396,t399,t4,t400,t401,t402,t403,t404,t405,t409,t410,t413,t414,t415,t416,t419,t428,t431,t433,t434,t435,t436,t437,t439,t440,t441,t442,t443,t444,t445,t446,t448,t449,t450,t452,t455,t464,t465,t466,t467,t470,t471,t475,t476,t477,t478,t479,t480,t481,t482,t483,t484,t486,t487,t488,t49,t490,t491,t492,t493,t494,t495,t497,t498,t499,t5,t501,t502,t507,t508,t511,t512,t513,t514,t515,t516,t519,t520,t524,t525,t527,t528,t529,t531,t532,t533,t534,t66,t67,t7,t75,t8,t83,t88,t89});
end
function B_fcn = ft_1(ct)
m_E = ct{1};
m_L = ct{2};
t10 = ct{3};
t11 = ct{4};
t12 = ct{5};
t126 = ct{6};
t128 = ct{7};
t13 = ct{8};
t160 = ct{9};
t163 = ct{10};
t179 = ct{11};
t187 = ct{12};
t188 = ct{13};
t196 = ct{14};
t197 = ct{15};
t198 = ct{16};
t199 = ct{17};
t200 = ct{18};
t201 = ct{19};
t202 = ct{20};
t203 = ct{21};
t204 = ct{22};
t205 = ct{23};
t206 = ct{24};
t214 = ct{25};
t215 = ct{26};
t218 = ct{27};
t219 = ct{28};
t226 = ct{29};
t227 = ct{30};
t228 = ct{31};
t229 = ct{32};
t230 = ct{33};
t231 = ct{34};
t232 = ct{35};
t233 = ct{36};
t234 = ct{37};
t235 = ct{38};
t236 = ct{39};
t237 = ct{40};
t239 = ct{41};
t240 = ct{42};
t249 = ct{43};
t250 = ct{44};
t253 = ct{45};
t254 = ct{46};
t262 = ct{47};
t263 = ct{48};
t264 = ct{49};
t265 = ct{50};
t266 = ct{51};
t267 = ct{52};
t268 = ct{53};
t269 = ct{54};
t270 = ct{55};
t271 = ct{56};
t288 = ct{57};
t289 = ct{58};
t293 = ct{59};
t294 = ct{60};
t310 = ct{61};
t311 = ct{62};
t318 = ct{63};
t319 = ct{64};
t32 = ct{65};
t322 = ct{66};
t323 = ct{67};
t324 = ct{68};
t325 = ct{69};
t326 = ct{70};
t328 = ct{71};
t329 = ct{72};
t330 = ct{73};
t331 = ct{74};
t332 = ct{75};
t333 = ct{76};
t335 = ct{77};
t336 = ct{78};
t337 = ct{79};
t338 = ct{80};
t339 = ct{81};
t340 = ct{82};
t341 = ct{83};
t342 = ct{84};
t343 = ct{85};
t344 = ct{86};
t345 = ct{87};
t346 = ct{88};
t347 = ct{89};
t348 = ct{90};
t349 = ct{91};
t350 = ct{92};
t351 = ct{93};
t352 = ct{94};
t353 = ct{95};
t354 = ct{96};
t355 = ct{97};
t356 = ct{98};
t357 = ct{99};
t358 = ct{100};
t359 = ct{101};
t360 = ct{102};
t361 = ct{103};
t362 = ct{104};
t364 = ct{105};
t365 = ct{106};
t366 = ct{107};
t367 = ct{108};
t368 = ct{109};
t369 = ct{110};
t370 = ct{111};
t371 = ct{112};
t372 = ct{113};
t373 = ct{114};
t374 = ct{115};
t375 = ct{116};
t376 = ct{117};
t377 = ct{118};
t378 = ct{119};
t379 = ct{120};
t380 = ct{121};
t381 = ct{122};
t382 = ct{123};
t383 = ct{124};
t384 = ct{125};
t385 = ct{126};
t386 = ct{127};
t390 = ct{128};
t391 = ct{129};
t392 = ct{130};
t393 = ct{131};
t394 = ct{132};
t396 = ct{133};
t399 = ct{134};
t4 = ct{135};
t400 = ct{136};
t401 = ct{137};
t402 = ct{138};
t403 = ct{139};
t404 = ct{140};
t405 = ct{141};
t409 = ct{142};
t410 = ct{143};
t413 = ct{144};
t414 = ct{145};
t415 = ct{146};
t416 = ct{147};
t419 = ct{148};
t428 = ct{149};
t431 = ct{150};
t433 = ct{151};
t434 = ct{152};
t435 = ct{153};
t436 = ct{154};
t437 = ct{155};
t439 = ct{156};
t440 = ct{157};
t441 = ct{158};
t442 = ct{159};
t443 = ct{160};
t444 = ct{161};
t445 = ct{162};
t446 = ct{163};
t448 = ct{164};
t449 = ct{165};
t450 = ct{166};
t452 = ct{167};
t455 = ct{168};
t464 = ct{169};
t465 = ct{170};
t466 = ct{171};
t467 = ct{172};
t470 = ct{173};
t471 = ct{174};
t475 = ct{175};
t476 = ct{176};
t477 = ct{177};
t478 = ct{178};
t479 = ct{179};
t480 = ct{180};
t481 = ct{181};
t482 = ct{182};
t483 = ct{183};
t484 = ct{184};
t486 = ct{185};
t487 = ct{186};
t488 = ct{187};
t49 = ct{188};
t490 = ct{189};
t491 = ct{190};
t492 = ct{191};
t493 = ct{192};
t494 = ct{193};
t495 = ct{194};
t497 = ct{195};
t498 = ct{196};
t499 = ct{197};
t5 = ct{198};
t501 = ct{199};
t502 = ct{200};
t507 = ct{201};
t508 = ct{202};
t511 = ct{203};
t512 = ct{204};
t513 = ct{205};
t514 = ct{206};
t515 = ct{207};
t516 = ct{208};
t519 = ct{209};
t520 = ct{210};
t524 = ct{211};
t525 = ct{212};
t527 = ct{213};
t528 = ct{214};
t529 = ct{215};
t531 = ct{216};
t532 = ct{217};
t533 = ct{218};
t534 = ct{219};
t66 = ct{220};
t67 = ct{221};
t7 = ct{222};
t75 = ct{223};
t8 = ct{224};
t83 = ct{225};
t88 = ct{226};
t89 = ct{227};
t535 = t10.*t11.*t13.*t66.*t88.*t201;
t536 = t10.*t11.*t13.*t66.*t88.*t202;
t537 = t10.*t11.*t13.*t66.*t88.*t203;
t538 = t10.*t11.*t13.*t66.*t88.*t204;
t539 = t10.*t11.*t13.*t66.*t88.*t205;
t540 = t10.*t11.*t13.*t66.*t88.*t206;
t541 = t10.*t11.*t13.*t66.*t89.*t200;
t542 = t10.*t11.*t13.*t66.*t89.*t201;
t543 = t10.*t11.*t13.*t66.*t89.*t204;
t544 = t10.*t11.*t13.*t66.*t89.*t205;
t545 = t10.*t11.*t13.*t66.*t89.*t206;
t546 = (t12.*t13.*t83.*t88.*t187)./2.0;
t547 = (t12.*t13.*t83.*t88.*t188)./2.0;
t548 = (t12.*t13.*t83.*t89.*t187)./2.0;
t549 = (t12.*t13.*t83.*t89.*t188)./2.0;
t550 = t10.*t11.*t13.*t66.*t88.*t218;
t551 = t10.*t11.*t13.*t66.*t88.*t219;
t552 = t10.*t11.*t13.*t66.*t89.*t219;
t553 = (t7.*t12.*t13.*t66.*t88.*t187)./2.0;
t554 = (t7.*t12.*t13.*t66.*t88.*t188)./2.0;
t555 = (t7.*t12.*t13.*t66.*t89.*t187)./2.0;
t556 = (t7.*t12.*t13.*t66.*t89.*t188)./2.0;
t558 = (t12.*t13.*t83.*t88.*t197)./2.0;
t559 = (t12.*t13.*t83.*t88.*t198)./2.0;
t561 = (t12.*t13.*t83.*t88.*t199)./2.0;
t562 = (t12.*t13.*t83.*t88.*t200)./2.0;
t563 = (t12.*t13.*t83.*t88.*t201)./2.0;
t564 = (t12.*t13.*t83.*t88.*t202)./2.0;
t565 = (t12.*t13.*t83.*t88.*t203)./2.0;
t566 = (t12.*t13.*t83.*t88.*t204)./2.0;
t567 = (t12.*t13.*t83.*t88.*t205)./2.0;
t568 = (t12.*t13.*t83.*t88.*t206)./2.0;
t569 = (t12.*t13.*t83.*t89.*t197)./2.0;
t570 = (t12.*t13.*t83.*t89.*t198)./2.0;
t572 = (t12.*t13.*t83.*t89.*t199)./2.0;
t573 = (t12.*t13.*t83.*t89.*t200)./2.0;
t574 = (t12.*t13.*t83.*t89.*t201)./2.0;
t575 = (t12.*t13.*t83.*t89.*t202)./2.0;
t576 = (t12.*t13.*t83.*t89.*t203)./2.0;
t577 = (t12.*t13.*t83.*t89.*t204)./2.0;
t578 = (t12.*t13.*t83.*t89.*t205)./2.0;
t579 = (t12.*t13.*t83.*t89.*t206)./2.0;
t580 = (t7.*t12.*t13.*t66.*t88.*t197)./2.0;
t581 = (t7.*t12.*t13.*t66.*t88.*t198)./2.0;
t583 = (t7.*t12.*t13.*t66.*t88.*t199)./2.0;
t584 = (t7.*t12.*t13.*t66.*t88.*t200)./2.0;
t585 = (t7.*t12.*t13.*t66.*t88.*t201)./2.0;
t586 = (t7.*t12.*t13.*t66.*t88.*t202)./2.0;
t587 = (t7.*t12.*t13.*t66.*t88.*t203)./2.0;
t588 = (t7.*t12.*t13.*t66.*t88.*t204)./2.0;
t589 = (t7.*t12.*t13.*t66.*t88.*t205)./2.0;
t590 = (t7.*t12.*t13.*t66.*t88.*t206)./2.0;
t591 = (t7.*t12.*t13.*t66.*t89.*t197)./2.0;
t592 = (t7.*t12.*t13.*t66.*t89.*t198)./2.0;
t593 = (t7.*t12.*t13.*t66.*t89.*t199)./2.0;
t594 = (t7.*t12.*t13.*t66.*t89.*t200)./2.0;
t595 = (t7.*t12.*t13.*t66.*t89.*t201)./2.0;
t596 = (t7.*t12.*t13.*t66.*t89.*t202)./2.0;
t597 = (t7.*t12.*t13.*t66.*t89.*t203)./2.0;
t598 = (t7.*t12.*t13.*t66.*t89.*t204)./2.0;
t599 = (t7.*t12.*t13.*t66.*t89.*t205)./2.0;
t600 = (t7.*t12.*t13.*t66.*t89.*t206)./2.0;
t608 = (t12.*t13.*t83.*t88.*t218)./2.0;
t612 = (t12.*t13.*t83.*t88.*t219)./2.0;
t615 = (t12.*t13.*t83.*t89.*t218)./2.0;
t619 = (t12.*t13.*t83.*t89.*t219)./2.0;
t622 = (t7.*t12.*t13.*t66.*t88.*t218)./2.0;
t626 = (t7.*t12.*t13.*t66.*t88.*t219)./2.0;
t627 = (t7.*t12.*t13.*t66.*t89.*t218)./2.0;
t628 = (t7.*t12.*t13.*t66.*t89.*t219)./2.0;
t251 = t8.*t214;
t252 = t8.*t215;
t272 = t8.*t226;
t273 = t8.*t227;
t274 = t8.*t228;
t275 = t8.*t229;
t276 = t8.*t230;
t277 = t8.*t231;
t278 = t8.*t232;
t279 = t8.*t233;
t280 = t8.*t234;
t281 = t8.*t235;
t290 = t8.*t239;
t291 = t8.*t240;
t295 = t11.*t249;
t296 = t11.*t250;
t297 = t11.*t262;
t298 = t11.*t263;
t299 = t11.*t264;
t300 = t11.*t265;
t301 = t11.*t266;
t302 = t11.*t267;
t303 = t11.*t268;
t304 = t11.*t269;
t305 = t11.*t270;
t306 = t11.*t271;
t308 = t11.*t288;
t309 = t11.*t289;
t363 = -t351;
t387 = -t359;
t388 = -t361;
t389 = -t362;
t557 = -t546;
t560 = -t547;
t571 = -t549;
t582 = -t554;
t601 = -t558;
t602 = -t559;
t603 = -t561;
t604 = -t562;
t605 = -t563;
t606 = -t564;
t607 = -t565;
t609 = -t566;
t610 = -t567;
t611 = -t568;
t613 = -t573;
t614 = -t574;
t616 = -t577;
t617 = -t578;
t618 = -t579;
t620 = -t584;
t621 = -t585;
t623 = -t588;
t624 = -t589;
t625 = -t590;
t629 = -t608;
t630 = -t612;
t631 = -t619;
t632 = -t626;
t633 = t13.*t49.*t214.*t322;
t634 = t13.*t49.*t215.*t322;
t635 = t13.*t49.*t226.*t336;
t636 = t13.*t49.*t227.*t337;
t637 = t13.*t49.*t231.*t336;
t638 = t13.*t49.*t232.*t337;
t639 = t13.*t49.*t228.*t339;
t640 = t13.*t49.*t229.*t340;
t641 = t13.*t49.*t230.*t341;
t642 = t13.*t49.*t233.*t339;
t643 = t13.*t49.*t234.*t340;
t644 = t13.*t49.*t235.*t341;
t645 = t13.*t49.*t239.*t347;
t646 = t13.*t49.*t240.*t347;
t647 = t13.*t66.*t249.*t346;
t648 = t13.*t66.*t250.*t346;
t649 = t13.*t66.*t262.*t357;
t650 = t13.*t66.*t263.*t358;
t651 = t13.*t66.*t267.*t357;
t652 = t13.*t66.*t268.*t358;
t653 = t13.*t66.*t264.*t384;
t654 = t13.*t66.*t265.*t385;
t655 = t13.*t66.*t266.*t386;
t656 = t13.*t66.*t269.*t384;
t657 = t13.*t66.*t270.*t385;
t658 = t13.*t66.*t271.*t386;
t659 = t13.*t66.*t288.*t404;
t660 = t13.*t66.*t289.*t404;
t307 = -t295;
t312 = -t297;
t313 = -t298;
t314 = -t299;
t315 = -t300;
t316 = -t301;
t321 = -t308;
t661 = t252+t323+t325+t353+t356;
t662 = t277+t323+t325+t364+t377;
t663 = t278+t323+t325+t365+t378;
t664 = t279+t323+t325+t366+t381;
t665 = t280+t323+t325+t369+t382;
t666 = t281+t323+t325+t370+t383;
t667 = t291+t323+t325+t390+t394;
t668 = t32+t251+t319+t324+t354+t392;
t669 = t32+t272+t319+t324+t367+t399;
t670 = t32+t273+t319+t324+t368+t400;
t671 = t32+t274+t319+t324+t371+t401;
t672 = t32+t275+t319+t324+t372+t402;
t673 = t32+t276+t319+t324+t373+t403;
t674 = t32+t290+t319+t324+t391+t428;
t675 = t296+t342+t345+t524+t528;
t676 = t302+t342+t345+t531+t541;
t677 = t303+t342+t345+t532+t542;
t678 = t304+t342+t345+t533+t543;
t679 = t305+t342+t345+t536+t544;
t680 = t306+t342+t345+t537+t545;
t681 = t309+t342+t345+t550+t552;
t689 = t328+t330+t332+t335+t396+t416+t452+t466+t634;
t690 = t67+t326+t329+t333+t338+t405+t419+t431+t455+t633;
t691 = t328+t330+t332+t335+t409+t440+t478+t490+t637;
t692 = t328+t330+t332+t335+t410+t441+t479+t491+t638;
t693 = t328+t330+t332+t335+t413+t442+t480+t493+t642;
t694 = t328+t330+t332+t335+t414+t445+t483+t494+t643;
t695 = t328+t330+t332+t335+t415+t446+t484+t495+t644;
t696 = t328+t330+t332+t335+t439+t476+t497+t499+t646;
t697 = t67+t326+t329+t333+t338+t433+t443+t464+t481+t635;
t698 = t67+t326+t329+t333+t338+t434+t444+t465+t482+t636;
t699 = t67+t326+t329+t333+t338+t435+t448+t467+t486+t639;
t700 = t67+t326+t329+t333+t338+t436+t449+t470+t487+t640;
t701 = t67+t326+t329+t333+t338+t437+t450+t471+t488+t641;
t702 = t67+t326+t329+t333+t338+t475+t477+t492+t498+t645;
t703 = t350+t363+t387+t389+t548+t553+t556+t560+t648;
t704 = t350+t363+t387+t389+t569+t580+t594+t604+t651;
t705 = t350+t363+t387+t389+t570+t581+t595+t605+t652;
t706 = t350+t363+t387+t389+t572+t583+t598+t609+t656;
t707 = t350+t363+t387+t389+t575+t586+t599+t610+t657;
t708 = t350+t363+t387+t389+t576+t587+t600+t611+t658;
t709 = t75+t349+t352+t360+t388+t555+t557+t571+t582+t647;
t710 = t350+t363+t387+t389+t615+t622+t628+t630+t660;
t711 = t75+t349+t352+t360+t388+t591+t601+t613+t620+t649;
t712 = t75+t349+t352+t360+t388+t592+t602+t614+t621+t650;
t713 = t75+t349+t352+t360+t388+t593+t603+t616+t623+t653;
t714 = t75+t349+t352+t360+t388+t596+t606+t617+t624+t654;
t715 = t75+t349+t352+t360+t388+t597+t607+t618+t625+t655;
t716 = t75+t349+t352+t360+t388+t627+t629+t631+t632+t659;
t682 = t11+t307+t343+t348+t502+t525;
t683 = t11+t312+t343+t348+t514+t534;
t684 = t11+t313+t343+t348+t515+t535;
t685 = t11+t314+t343+t348+t516+t538;
t686 = t11+t315+t343+t348+t519+t539;
t687 = t11+t316+t343+t348+t520+t540;
t688 = t11+t321+t343+t348+t529+t551;
et1 = m_E.*(t4.*t128.*(-1.0./1.2e+1)+t5.*(t8+t318-t319-t354+t355+t32.*t214).*(-t11+t295+t344+t501+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t187)+t5.*t661.*t675).*(-1.0./2.0)-(m_L.*(t4.*t179.*(-3.0./6.4e+1)+t5.*(t8+t318-t319-t368+t375+t32.*t227).*(-t11+t298+t344+t508+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t198)+t5.*t663.*t677))./1.2e+1-(m_L.*(t4.*t163.*(-1.0./1.92e+2)+t5.*(t8+t318-t319-t367+t374+t32.*t226).*(-t11+t297+t344+t507+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t197)+t5.*t662.*t676))./1.2e+1;
et2 = m_L.*(t4.*t196.*(-5.787037037037037e-4)+t5.*(t8+t318-t319-t371+t376+t32.*t228).*(-t11+t299+t344+t511+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t199)+t5.*t664.*t678).*(-1.0./1.2e+1)-(m_L.*(t4.*t236.*(-1.446759259259259e-2)+t5.*(t8+t318-t319-t372+t379+t32.*t229).*(-t11+t300+t344+t512+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t202)+t5.*t665.*t679))./1.2e+1-(m_L.*(t4.*t237.*(-2.835648148148148e-2)+t5.*(t8+t318-t319-t373+t380+t32.*t230).*(-t11+t301+t344+t513+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t203)+t5.*t666.*t680))./1.2e+1;
et3 = m_L.*(t4.*t253.*(-7.002314814814815e-2)+t5.*(t8+t318-t319-t391+t393+t32.*t239).*(-t11+t308+t344+t527+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t218)+t5.*t667.*t681).*(-1.0./1.2e+1);
et4 = (m_E.*(t160+t5.*t690.*(-t11+t295+t344+t501+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t187)-t5.*t675.*t689))./2.0+(m_L.*(t254+t5.*t697.*(-t11+t297+t344+t507+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t197)-t5.*t676.*t691))./1.2e+1+(m_L.*(t293+t5.*t698.*(-t11+t298+t344+t508+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t198)-t5.*t677.*t692))./1.2e+1+(m_L.*(t294+t5.*t699.*(-t11+t299+t344+t511+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t199)-t5.*t678.*t693))./1.2e+1+(m_L.*(t310+t5.*t700.*(-t11+t300+t344+t512+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t202)-t5.*t679.*t694))./1.2e+1;
et5 = (m_L.*(t311+t5.*t701.*(-t11+t301+t344+t513+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t203)-t5.*t680.*t695))./1.2e+1+(m_L.*(t331+t5.*t702.*(-t11+t308+t344+t527+t10.*t11.*t13.*t66.*t88.*t126+t10.*t11.*t13.*t66.*t89.*t218)-t5.*t681.*t696))./1.2e+1;
et6 = m_E.*(-t160+t5.*t661.*t703+t5.*t709.*(t8+t318-t319-t354+t355+t32.*t214)).*(-1.0./2.0)-(m_L.*(-t254+t5.*t662.*t704+t5.*t711.*(t8+t318-t319-t367+t374+t32.*t226)))./1.2e+1-(m_L.*(-t293+t5.*t663.*t705+t5.*t712.*(t8+t318-t319-t368+t375+t32.*t227)))./1.2e+1-(m_L.*(-t294+t5.*t664.*t706+t5.*t713.*(t8+t318-t319-t371+t376+t32.*t228)))./1.2e+1-(m_L.*(-t310+t5.*t665.*t707+t5.*t714.*(t8+t318-t319-t372+t379+t32.*t229)))./1.2e+1;
et7 = m_L.*(-t311+t5.*t666.*t708+t5.*t715.*(t8+t318-t319-t373+t380+t32.*t230)).*(-1.0./1.2e+1)-(m_L.*(-t331+t5.*t667.*t710+t5.*t716.*(t8+t318-t319-t391+t393+t32.*t239)))./1.2e+1;
et8 = (m_E.*((t4.*t128)./4.8e+1-t5.*t689.*t703+t5.*t690.*t709))./2.0+(m_L.*(t4.*t179.*6.591796875e-3-t5.*t692.*t705+t5.*t698.*t712))./1.2e+1+(m_L.*((t4.*t163)./1.2288e+4-t5.*t691.*t704+t5.*t697.*t711))./1.2e+1+(m_L.*((t4.*t196)./9.95328e+5-t5.*t693.*t706+t5.*t699.*t713))./1.2e+1+(m_L.*(t4.*t236.*6.279337062757202e-4-t5.*t694.*t707+t5.*t700.*t714))./1.2e+1+(m_L.*(t4.*t237.*2.412270126028807e-3-t5.*t695.*t708+t5.*t701.*t715))./1.2e+1;
et9 = (m_L.*(t4.*t253.*1.470972382973251e-2-t5.*t696.*t710+t5.*t702.*t716))./1.2e+1;
B_fcn = reshape([et1+et2+et3,et6+et7,et4+et5,et8+et9],[2,2]);
end
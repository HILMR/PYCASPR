function u_c = Discriminant_deg_4(in1,in2,in3,in4,u0)
%DISCRIMINANT_DEG_4
%    U_C = DISCRIMINANT_DEG_4(IN1,IN2,IN3,IN4,U0)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Dec-2020 17:11:15

u11 = in4(:,1);
u12 = in4(:,2);
u21 = in3(:,1);
u22 = in3(:,2);
u23 = in3(:,3);
u31 = in2(:,1);
u32 = in2(:,2);
u33 = in2(:,3);
u34 = in2(:,4);
u41 = in1(:,1);
u42 = in1(:,2);
u43 = in1(:,3);
u44 = in1(:,4);
u45 = in1(:,5);
t2 = u31.^2;
t3 = u0.^2;
t4 = u41.^2;
t5 = u11.^2;
t6 = u21.^2;
t7 = t6.^2;
t8 = t5.^2;
t9 = t5.*u0.*u22;
t10 = u0.*u11.*u12.*u21.*2.0;
t11 = t9+t10;
t12 = u12.^2;
t13 = u22.^2;
t14 = u32.^2;
t15 = t3.*u11.*u32;
t16 = t3.*u12.*u31;
t17 = t15+t16;
t18 = u42.*u45.*2.0;
t19 = u43.*u44.*2.0;
t20 = u31.*u33.*2.0;
t21 = t14+t20;
t22 = t21.*u31;
t23 = t14.*u31.*2.0;
t24 = t2.*u33;
t25 = t22+t23+t24;
t26 = u42.^2;
t27 = u21.*u23.*2.0;
t28 = t13+t27;
t29 = t6.*u11.*u12.*u21.*2.0;
t30 = t5.*t6.*u22.*3.0;
t31 = u23.^2;
t32 = u41.*u42.*2.0;
t33 = t18+t19+t32;
t34 = t5.*u11.*u22;
t35 = t5.*u12.*u21.*3.0;
t36 = t34+t35;
t37 = t6.*u0.*u12;
t38 = u0.*u11.*u21.*u22.*2.0;
t39 = t37+t38;
t40 = t5.*u11.*u23;
t41 = t12.*u11.*u21.*3.0;
t42 = t5.*u12.*u22.*3.0;
t43 = t40+t41+t42;
t44 = t12.*u12.*u21;
t45 = t12.*u11.*u22.*3.0;
t46 = t5.*u12.*u23.*3.0;
t47 = t44+t45+t46;
t48 = t47.*u33;
t49 = t43.*u34;
t50 = t36.*u31;
t51 = t12.*u12.*u22;
t52 = t12.*u11.*u23.*3.0;
t53 = t51+t52;
t54 = t53.*u32;
t55 = t5.*u11.*u21.*u32;
t56 = t12.*u12.*u23.*u31;
t57 = u32.*u34.*2.0;
t58 = u33.^2;
t59 = t57+t58;
t60 = t3.*t59.*u22;
t61 = t2.*t3.*u22;
t62 = u31.*u34.*2.0;
t63 = u32.*u33.*2.0;
t64 = t62+t63;
t65 = t3.*t64.*u23;
t66 = t3.*u21.*u31.*u32.*2.0;
t67 = t3.*u21.*u33.*u34.*2.0;
t68 = t28.*u22;
t69 = u21.*u22.*u23.*4.0;
t70 = t68+t69;
t71 = t12.*t70;
t72 = t28.*u23;
t73 = t31.*u21;
t74 = t13.*u23.*2.0;
t75 = t72+t73+t74;
t76 = t75.*u11.*u12.*2.0;
t77 = t5.*t31.*u22.*3.0;
t78 = t28.*u0.*u11;
t79 = u0.*u12.*u21.*u22.*2.0;
t80 = t78+t79;
t81 = u41.*u43.*2.0;
t82 = t26+t81;
t83 = t80.*u34;
t84 = t28.*u0.*u12;
t85 = u0.*u11.*u22.*u23.*2.0;
t86 = t84+t85;
t87 = t86.*u33;
t88 = t39.*u31;
t89 = t31.*u0.*u11;
t90 = u0.*u12.*u22.*u23.*2.0;
t91 = t89+t90;
t92 = t91.*u32;
t93 = t6.*u0.*u11.*u32;
t94 = t31.*u0.*u12.*u31;
t95 = t12.*t64.*u0;
t96 = t59.*u0.*u11.*u12.*2.0;
t97 = t2.*u0.*u11.*u12.*2.0;
t98 = t5.*u0.*u31.*u32.*2.0;
t99 = t5.*u0.*u33.*u34.*2.0;
t100 = t18+t19;
t101 = t28.*u21;
t102 = t13.*u21.*2.0;
t103 = t6.*u23;
t104 = t101+t102+t103;
t105 = u0.*u11.*u22;
t106 = u0.*u12.*u21;
t107 = t105+t106;
t108 = t70.*u23;
t109 = t75.*u22;
t110 = t31.*u21.*u22.*3.0;
t111 = t108+t109+t110;
t112 = t6.*u11.*u12.*2.0;
t113 = t5.*u21.*u22.*2.0;
t114 = t112+t113;
t115 = t12.*u0.*u21;
t116 = t5.*u0.*u23;
t117 = u0.*u11.*u12.*u22.*2.0;
t118 = t115+t116+t117;
t119 = u41.*u44.*2.0;
t120 = u42.*u43.*2.0;
t121 = t119+t120;
t122 = t3.*u11.*u33;
t123 = t3.*u12.*u32;
t124 = t122+t123;
t125 = t29+t30+t71+t76+t77;
t126 = t48+t49+t54+t56;
t127 = u34.^2;
t128 = t60+t65+t67;
t129 = u43.*u45.*2.0;
t130 = u44.^2;
t131 = t129+t130;
t132 = t48+t49+t50+t54+t55+t56;
t133 = t43.*u31;
t134 = t47.*u34;
t135 = t36.*u32;
t136 = t53.*u33;
t137 = t5.*u11.*u21.*u33;
t138 = t12.*u12.*u23.*u32;
t139 = t60+t61+t65+t66+t67;
t140 = t95+t96+t99;
t141 = t3.*t21.*u21;
t142 = t3.*t59.*u23;
t143 = t2.*t3.*u23;
t144 = t3.*t127.*u21;
t145 = t3.*u22.*u31.*u32.*2.0;
t146 = t3.*u22.*u33.*u34.*2.0;
t147 = t71+t76+t77;
t148 = t83+t87+t92+t94;
t149 = t26+t81+t129+t130;
t150 = t6.*t13.*3.0;
t151 = t6.*u21.*u23;
t152 = t104.*u21;
t153 = t150+t151+t152;
t154 = t111.*u0;
t155 = t6.*u0.*u21.*u22.*4.0;
t156 = t154+t155;
t157 = t83+t87+t88+t92+t93+t94;
t158 = t80.*u31;
t159 = t86.*u34;
t160 = t39.*u32;
t161 = t91.*u33;
t162 = t6.*u0.*u11.*u33;
t163 = t31.*u0.*u12.*u32;
t164 = t95+t96+t97+t98+t99;
t165 = t5.*t21.*u0;
t166 = t12.*t59.*u0;
t167 = t2.*t12.*u0;
t168 = t5.*t127.*u0;
t169 = u0.*u11.*u12.*u31.*u32.*4.0;
t170 = u0.*u11.*u12.*u33.*u34.*4.0;
t171 = t6.*t12.*u21;
t172 = t5.*t31.*u23;
t173 = t5.*t104;
t174 = t12.*t75;
t175 = t6.*u11.*u12.*u22.*6.0;
t176 = t31.*u11.*u12.*u22.*6.0;
t177 = t21.*u32;
t178 = t2.*u34;
t179 = t64.*u31;
t180 = u31.*u32.*u33.*2.0;
t181 = t177+t178+t179+t180;
t182 = t13.*t31.*3.0;
t183 = t31.*u21.*u23;
t184 = t75.*u23;
t185 = t182+t183+t184;
t186 = u0.*u11.*u23;
t187 = u0.*u12.*u22;
t188 = t186+t187;
t189 = t6.*t12;
t190 = t5.*t28;
t191 = u11.*u12.*u21.*u22.*4.0;
t192 = t189+t190+t191;
t193 = t12.*u0.*u22;
t194 = u0.*u11.*u12.*u23.*2.0;
t195 = t193+t194;
t196 = t172+t174+t176;
t197 = t3.*u11.*u34;
t198 = t3.*u12.*u33;
t199 = t197+t198;
t200 = t5.*t70;
t201 = t104.*u11.*u12.*2.0;
t202 = t6.*t12.*u22.*3.0;
t203 = u44.*u45.*2.0;
t204 = t119+t120+t203;
t205 = t134+t136+t138;
t206 = t142+t144+t146;
t207 = t159+t161+t163;
t208 = t166+t168+t170;
t209 = t133+t134+t135+t136+t137+t138;
t210 = t47.*u31;
t211 = t43.*u32;
t212 = t36.*u33;
t213 = t53.*u34;
t214 = t5.*u11.*u21.*u34;
t215 = t12.*u12.*u23.*u33;
t216 = t3.*t21.*u22;
t217 = t3.*t127.*u22;
t218 = t3.*t64.*u21;
t219 = t3.*u23.*u31.*u32.*2.0;
t220 = t3.*u23.*u33.*u34.*2.0;
t221 = t141+t142+t143+t144+t145+t146;
t222 = t153.*u0;
t223 = t185.*u0;
t224 = t222+t223;
t225 = t21.*u33;
t226 = t59.*u31;
t227 = t64.*u32;
t228 = u31.*u32.*u34.*2.0;
t229 = t225+t226+t227+t228;
t230 = t70.*u21;
t231 = t104.*u22;
t232 = t6.*u22.*u23.*3.0;
t233 = t230+t231+t232;
t234 = t158+t159+t160+t161+t162+t163;
t235 = t80.*u32;
t236 = t86.*u31;
t237 = t39.*u33;
t238 = t91.*u34;
t239 = t6.*u0.*u11.*u34;
t240 = t31.*u0.*u12.*u33;
t241 = t5.*t64.*u0;
t242 = t21.*u0.*u11.*u12.*2.0;
t243 = t127.*u0.*u11.*u12.*2.0;
t244 = t12.*u0.*u31.*u32.*2.0;
t245 = t12.*u0.*u33.*u34.*2.0;
t246 = t165+t166+t167+t168+t169+t170;
t247 = t31.*u11.*u12.*u23.*2.0;
t248 = t12.*t31.*u22.*3.0;
t249 = t171+t172+t173+t174+t175+t176;
t250 = u41.*u45.*2.0;
t251 = u42.*u44.*2.0;
t252 = u43.^2;
t253 = t250+t251+t252;
t254 = t7.*u0.*u41.*1.6e1;
t255 = t12.*u21.*u22.*2.0;
t256 = t5.*u22.*u23.*2.0;
t257 = t28.*u11.*u12.*2.0;
t258 = t255+t256+t257;
t259 = t5.*u11.*u21.*u31.*u41.*1.8e1;
t260 = t4.*t5.*u0.*u21.*1.44e2;
t261 = t2.*t3.*u21.*u41.*1.44e2;
t262 = t70.*u22;
t263 = t104.*u23;
t264 = t75.*u21;
t265 = t262+t263+t264;
t266 = t3.*t6;
t267 = t3.*t31;
t268 = t266+t267;
t269 = t12.^2;
t270 = u41.*1.8e1;
t271 = u45.*1.8e1;
t272 = t270+t271;
t273 = u41.*1.44e2;
t274 = u45.*1.44e2;
t275 = t273+t274;
t276 = t80.*u33;
t277 = t86.*u32;
t278 = t39.*u34;
t279 = t91.*u31;
t280 = t5.*u0.*u21;
t281 = t12.*u0.*u23;
t282 = t280+t281;
t283 = t3.*u11.*u31;
t284 = t3.*u12.*u34;
t285 = t283+t284;
t286 = t200+t201+t202+t247+t248;
t287 = t61+t66;
t288 = t287.*u41.*1.44e2;
t289 = t217+t220;
t290 = t88+t93;
t291 = t238+t240;
t292 = t50+t55;
t293 = t292.*u41.*1.8e1;
t294 = t213+t215;
t295 = u41.*1.6e1;
t296 = u45.*1.6e1;
t297 = t295+t296;
t298 = u41.*8.0e1;
t299 = u45.*8.0e1;
t300 = t298+t299;
t301 = u41.*6.0;
t302 = u45.*6.0;
t303 = t301+t302;
t304 = t97+t98;
t305 = t243+t245;
t306 = t12.*t104;
t307 = t5.*t75;
t308 = t70.*u11.*u12.*2.0;
t309 = t47.*u32;
t310 = t43.*u33;
t311 = t53.*u31;
t312 = t36.*u34;
t313 = t210+t211+t212+t213+t214+t215;
t314 = t216+t217+t218+t219+t220;
t315 = t3.*t21.*u23;
t316 = t3.*t59.*u21;
t317 = t3.*t64.*u22;
t318 = t8+t269;
t319 = u45.^2;
t320 = t250+t251+t252+t319;
t321 = t12.*t21.*u0;
t322 = t5.*t59.*u0;
t323 = t64.*u0.*u11.*u12.*2.0;
t324 = t233.*u0;
t325 = t31.*u0.*u22.*u23.*4.0;
t326 = t324+t325;
t327 = t235+t236+t237+t238+t239+t240;
t328 = t241+t242+t243+t244+t245;
t329 = t29+t30;
t330 = t247+t248;
t331 = u41.*4.0;
t332 = u45.*4.0;
t333 = t331+t332;
t334 = t21.*u34;
t335 = t59.*u32;
t336 = t64.*u33;
t337 = u31.*u33.*u34.*2.0;
t338 = t334+t335+t336+t337;
t339 = t31.^2;
t340 = t5.*t31;
t341 = t12.*t28;
t342 = u11.*u12.*u22.*u23.*4.0;
t343 = t340+t341+t342;
t344 = t6.*u0.*u21.*u22.*u41.*6.4e1;
t345 = t5.*u0.*u21.*u41.*u42.*2.88e2;
t346 = t265.*u0;
t347 = t339.*u0;
t348 = t346+t347;
t349 = t171+t173+t175;
t350 = t31.*u0.*u12.*u34;
t351 = t276+t277+t278+t279+t350;
t352 = t12.*t31.*u23;
t353 = t306+t307+t308+t352;
t354 = t133+t135+t137;
t355 = t354.*u41.*1.8e1;
t356 = t141+t143+t145;
t357 = t356.*u41.*1.44e2;
t358 = t12.*u12.*u23.*u34;
t359 = t309+t310+t311+t312+t358;
t360 = t158+t160+t162;
t361 = t165+t167+t169;
t362 = t3.*t127.*u23;
t363 = t315+t316+t317+t362;
t364 = t140.*u42.*6.0;
t365 = t147.*u42.*4.0;
t366 = t148.*u42.*8.0e1;
t367 = t12.*t127.*u0;
t368 = t321+t322+t323+t367;
t369 = t59.*u33;
t370 = t127.*u31;
t371 = t64.*u34;
t372 = u32.*u33.*u34.*2.0;
t373 = t369+t370+t371+t372;
t374 = t17.*t100.*1.92e2;
t375 = t153.*u0.*u41.*1.6e1;
t376 = t5.*t100.*u11.*u12.*1.08e2;
t377 = t31.*u11.*u12.*2.0;
t378 = t12.*u22.*u23.*2.0;
t379 = t377+t378;
t380 = t3.*t100.*u21.*u22.*2.56e2;
t381 = t5.*t82.*u0.*u21.*1.44e2;
t382 = t196.*u42.*4.0;
t383 = t17.*t131.*1.92e2;
t384 = t207.*u42.*8.0e1;
t385 = t210+t211+t212+t214;
t386 = t385.*u41.*1.8e1;
t387 = t208.*u42.*6.0;
t388 = t216+t218+t219;
t389 = t388.*u41.*1.44e2;
t390 = t241+t242+t244;
t391 = t140.*u43.*6.0;
t392 = t200+t201+t202;
t393 = t147.*u43.*4.0;
t394 = t235+t236+t237+t239;
t395 = t148.*u43.*8.0e1;
t396 = t59.*u34;
t397 = t127.*u32;
t398 = t58.*u34.*2.0;
t399 = t396+t397+t398;
t400 = t100.*t124.*1.92e2;
t401 = t5.*t131.*u11.*u12.*1.08e2;
t402 = t3.*t28.*t100.*1.28e2;
t403 = t233.*u0.*u41.*1.6e1;
t404 = t5.*t12.*t100.*1.62e2;
t405 = t5.*t121.*u0.*u21.*1.44e2;
t406 = t3.*t131.*u21.*u22.*2.56e2;
t407 = t196.*u43.*4.0;
t408 = t315+t316+t317;
t409 = t408.*u41.*1.44e2;
t410 = t321+t322+t323;
t411 = t291.*u42.*8.0e1;
t412 = t305.*u42.*6.0;
t413 = t309+t310+t311+t312;
t414 = t413.*u41.*1.8e1;
t415 = t306+t307+t308;
t416 = t124.*t131.*1.92e2;
t417 = t207.*u43.*8.0e1;
t418 = t208.*u43.*6.0;
t419 = t276+t277+t278+t279;
t420 = t140.*u44.*6.0;
t421 = t147.*u44.*4.0;
t422 = t148.*u44.*8.0e1;
t423 = t330.*u42.*4.0;
t424 = t100.*t199.*1.92e2;
t425 = t3.*t28.*t131.*1.28e2;
t426 = t5.*t12.*t131.*1.62e2;
t427 = t12.*t100.*u11.*u12.*1.08e2;
t428 = t265.*u0.*u41.*1.6e1;
t429 = t17.*u44.*u45.*3.84e2;
t430 = t3.*t100.*u22.*u23.*2.56e2;
t431 = t5.*u11.*u12.*u44.*u45.*2.16e2;
t432 = t5.*t253.*u0.*u21.*1.44e2;
t433 = t3.*u21.*u22.*u44.*u45.*5.12e2;
t434 = t196.*u44.*4.0;
t435 = t100.*t269.*2.7e1;
t436 = t291.*u43.*8.0e1;
t437 = t305.*u43.*6.0;
t438 = t131.*t199.*1.92e2;
t439 = t207.*u44.*8.0e1;
t440 = t208.*u44.*6.0;
t441 = t140.*u45.*6.0;
t442 = t147.*u45.*4.0;
t443 = t148.*u45.*8.0e1;
t444 = t17.*t319.*1.92e2;
t445 = t330.*u43.*4.0;
t446 = t12.*t131.*u11.*u12.*1.08e2;
t447 = t5.*t319.*u11.*u12.*1.08e2;
t448 = t12.*t31.*u23.*u42.*4.0;
t449 = t124.*u44.*u45.*3.84e2;
t450 = t3.*t31.*t100.*1.28e2;
t451 = t3.*t100.*u12.*u34.*1.92e2;
t452 = t3.*t28.*u44.*u45.*2.56e2;
t453 = t3.*t131.*u22.*u23.*2.56e2;
t454 = t12.*t127.*u0.*u42.*6.0;
t455 = t3.*t319.*u21.*u22.*2.56e2;
t456 = t5.*t12.*u44.*u45.*3.24e2;
t457 = t31.*u0.*u12.*u34.*u42.*8.0e1;
t458 = t196.*u45.*4.0;
t459 = t291.*u44.*8.0e1;
t460 = t305.*u44.*6.0;
t461 = t207.*u45.*8.0e1;
t462 = t208.*u45.*6.0;
t463 = t131.*t269.*2.7e1;
t464 = t124.*t319.*1.92e2;
t465 = t330.*u44.*4.0;
t466 = t12.*t31.*u23.*u43.*4.0;
t467 = t3.*t28.*t319.*1.28e2;
t468 = t3.*t31.*t131.*1.28e2;
t469 = t5.*t12.*t319.*1.62e2;
t470 = t199.*u44.*u45.*3.84e2;
t471 = t12.*u11.*u12.*u44.*u45.*2.16e2;
t472 = t3.*t131.*u12.*u34.*1.92e2;
t473 = t12.*t127.*u0.*u43.*6.0;
t474 = t31.*u0.*u12.*u34.*u43.*8.0e1;
t475 = t3.*u22.*u23.*u44.*u45.*5.12e2;
t476 = t291.*u45.*8.0e1;
t477 = t305.*u45.*6.0;
t478 = t199.*t319.*1.92e2;
t479 = t330.*u45.*4.0;
t480 = t269.*u44.*u45.*5.4e1;
t481 = t12.*t319.*u11.*u12.*1.08e2;
t482 = t12.*t31.*u23.*u44.*4.0;
t483 = t12.*t127.*u0.*u44.*6.0;
t484 = t3.*t319.*u22.*u23.*2.56e2;
t485 = t3.*t31.*u44.*u45.*2.56e2;
t486 = t31.*u0.*u12.*u34.*u44.*8.0e1;
t487 = t3.*u12.*u34.*u44.*u45.*3.84e2;
t488 = t269.*t319.*2.7e1;
t489 = t12.*t31.*u23.*u45.*4.0;
t490 = t3.*t31.*t319.*1.28e2;
t491 = t12.*t127.*u0.*u45.*6.0;
t492 = t3.*t319.*u12.*u34.*1.92e2;
t493 = t31.*u0.*u12.*u34.*u45.*8.0e1;
u_c = [t254+t259+t260+t261-t4.*t8.*2.7e1-t2.^2.*t3.*2.7e1+t2.*t5.*t6-t3.*t4.*t6.*1.28e2-t2.*t6.*u0.*u21.*4.0-t2.*t5.*u0.*u41.*6.0+t3.*t4.*u0.*u41.*2.56e2-t2.*t5.*u11.*u31.*4.0-t3.*t4.*u11.*u31.*1.92e2-t5.*t6.*u21.*u41.*4.0+t2.*u0.*u11.*u21.*u31.*1.8e1-t6.*u0.*u11.*u31.*u41.*8.0e1,t288+t293+t344+t345+t4.*t11.*1.44e2-t4.*t17.*1.92e2+t2.*t114-t290.*u41.*8.0e1-t304.*u41.*6.0-t329.*u41.*4.0+t2.*t107.*u31.*1.8e1+t7.*u0.*u42.*1.6e1-t8.*u41.*u42.*5.4e1-t2.*t6.*u0.*u22.*1.2e1-t4.*t5.*u11.*u12.*1.08e2-t2.*t5.*u0.*u42.*6.0+t3.*t4.*u0.*u42.*7.68e2-t2.*t5.*u11.*u32.*1.2e1-t2.*t5.*u12.*u31.*1.2e1-t3.*t4.*u21.*u22.*2.56e2+t2.*t3.*u21.*u42.*1.44e2-t2.*t3.*u31.*u32.*1.08e2-t5.*t6.*u21.*u42.*4.0+t5.*t6.*u31.*u32.*2.0-t3.*t6.*u41.*u42.*2.56e2+t2.*u0.*u11.*u21.*u32.*5.4e1-t6.*u0.*u11.*u31.*u42.*8.0e1-t6.*u0.*u21.*u31.*u32.*8.0+t5.*u11.*u21.*u31.*u42.*1.8e1-t3.*u11.*u31.*u41.*u42.*3.84e2,t355+t357+t364+t365+t366+t374+t375+t376+t380+t381+t11.*t33.*1.44e2-t17.*t33.*1.92e2-t8.*t82.*2.7e1-t11.*t100.*1.44e2+t4.*t118.*1.44e2-t4.*t124.*1.92e2+t2.*t192-t125.*u42.*4.0-t126.*u42.*1.8e1-t128.*u42.*1.44e2+t132.*u42.*1.8e1+t139.*u42.*1.44e2+t156.*u42.*1.6e1-t157.*u42.*8.0e1-t164.*u42.*6.0-t349.*u41.*4.0-t360.*u41.*8.0e1-t361.*u41.*6.0-t3.*(t2.*t14.*3.0+t25.*u31+t2.*u31.*u33).*2.7e1+t3.*u0.*(t4.*u43+t26.*u41.*2.0+t82.*u41).*2.56e2-t4.*t5.*t12.*1.62e2+t5.*t6.*t21-t3.*t4.*t28.*1.28e2-t3.*t6.*t82.*1.28e2-t5.*t25.*u11.*4.0-t2.*t104.*u0.*4.0+t2.*t107.*u32.*5.4e1+t2.*t188.*u31.*1.8e1+t7.*u0.*u43.*1.6e1-t111.*u0.*u42.*1.6e1+t114.*u31.*u32.*2.0-t6.*t21.*u0.*u21.*4.0-t2.*t5.*u0.*u43.*6.0-t2.*t5.*u12.*u32.*3.6e1-t2.*t12.*u11.*u31.*1.2e1-t5.*t33.*u11.*u12.*1.08e2+t2.*t3.*u21.*u43.*1.44e2-t5.*t6.*u21.*u43.*4.0-t3.*t33.*u21.*u22.*2.56e2-t3.*t82.*u11.*u31.*1.92e2+t25.*u0.*u11.*u21.*1.8e1-t6.*u0.*u11.*u31.*u43.*8.0e1-t6.*u0.*u22.*u31.*u32.*2.4e1+t5.*u11.*u21.*u31.*u43.*1.8e1,t382+t383+t384+t386+t387+t389+t391+t393+t395+t400+t401+t402+t403+t404+t405+t406-t3.*(t25.*u32+t181.*u31+t2.*u31.*u34+t2.*u32.*u33.*3.0).*2.7e1-t8.*t121.*2.7e1+t25.*t107.*1.8e1+t21.*t114-t11.*t131.*1.44e2+t33.*t118.*1.44e2-t33.*t124.*1.92e2+t11.*t149.*1.44e2-t17.*t149.*1.92e2+t4.*t195.*1.44e2-t4.*t199.*1.92e2-t100.*t118.*1.44e2+t2.*t258-t125.*u43.*4.0-t126.*u43.*1.8e1-t128.*u43.*1.44e2+t132.*u43.*1.8e1+t139.*u43.*1.44e2+t156.*u43.*1.6e1-t157.*u43.*8.0e1-t164.*u43.*6.0-t205.*u42.*1.8e1-t206.*u42.*1.44e2+t209.*u42.*1.8e1+t221.*u42.*1.44e2+t224.*u42.*1.6e1-t234.*u42.*8.0e1-t246.*u42.*6.0-t249.*u42.*4.0-t390.*u41.*6.0-t392.*u41.*4.0-t394.*u41.*8.0e1+t3.*u0.*(t4.*u44+t82.*u42+t121.*u41+u41.*u42.*u43.*2.0).*2.56e2-t5.*t12.*t33.*1.62e2-t3.*t28.*t33.*1.28e2+t5.*t6.*t64-t3.*t6.*t121.*1.28e2-t5.*t25.*u12.*1.2e1-t2.*t70.*u0.*4.0-t5.*t181.*u11.*4.0+t2.*t188.*u32.*5.4e1+t7.*u0.*u44.*1.6e1-t111.*u0.*u43.*1.6e1-t185.*u0.*u42.*1.6e1+t192.*u31.*u32.*2.0-t4.*t12.*u11.*u12.*1.08e2-t6.*t21.*u0.*u22.*1.2e1-t2.*t5.*u0.*u44.*6.0-t3.*t4.*u22.*u23.*2.56e2-t2.*t12.*u11.*u32.*3.6e1-t2.*t12.*u12.*u31.*4.0+t2.*t3.*u21.*u44.*1.44e2-t5.*t6.*u21.*u44.*4.0-t6.*t64.*u0.*u21.*4.0-t3.*t121.*u11.*u31.*1.92e2-t5.*t149.*u11.*u12.*1.08e2-t3.*t149.*u21.*u22.*2.56e2-t104.*u0.*u31.*u32.*8.0+t181.*u0.*u11.*u21.*1.8e1+t2.*u0.*u12.*u23.*u31.*1.8e1-t6.*u0.*u11.*u31.*u44.*8.0e1+t5.*u11.*u21.*u31.*u44.*1.8e1,-t254-t259-t260-t261+t407+t409+t411+t412+t414+t416+t417+t418+t420+t421+t422+t423+t424+t425+t426+t427+t428+t429+t430+t431+t432+t433+t4.*t8.*2.7e1+t64.*t114+t21.*t192+t25.*t188.*1.8e1+t11.*t204.*1.44e2-t17.*t204.*1.92e2+t33.*t195.*1.44e2-t33.*t199.*1.92e2-t118.*t131.*1.44e2-t8.*t253.*2.7e1+t118.*t149.*1.44e2-t4.*t268.*1.28e2-t124.*t149.*1.92e2+t4.*t282.*1.44e2+t107.*t181.*1.8e1-t4.*t285.*1.92e2-t100.*t195.*1.44e2-t4.*t318.*2.7e1+t2.*t343-t125.*u44.*4.0-t126.*u44.*1.8e1-t128.*u44.*1.44e2+t132.*u44.*1.8e1+t139.*u44.*1.44e2+t156.*u44.*1.6e1-t157.*u44.*8.0e1-t164.*u44.*6.0-t205.*u43.*1.8e1-t206.*u43.*1.44e2+t209.*u43.*1.8e1+t221.*u43.*1.44e2+t224.*u43.*1.6e1-t234.*u43.*8.0e1-t246.*u43.*6.0-t249.*u43.*4.0-t286.*u42.*4.0-t289.*u42.*1.44e2-t294.*u42.*1.8e1+t313.*u42.*1.8e1+t314.*u42.*1.44e2+t326.*u42.*1.6e1-t327.*u42.*8.0e1-t328.*u42.*6.0-t410.*u41.*6.0-t415.*u41.*4.0-t419.*u41.*8.0e1-t3.*(t25.*u33+t181.*u32+t229.*u31+t2.*u32.*u34.*3.0).*2.7e1+t3.*u0.*(t4.*u45+t82.*u43+t121.*u42+t253.*u41+u41.*u42.*u44.*2.0).*2.56e2+t3.*t4.*t6.*1.28e2+t5.*t6.*t59-t5.*t12.*t149.*1.62e2-t3.*t28.*t149.*1.28e2-t3.*t6.*t253.*1.28e2-t12.*t25.*u11.*1.2e1-t2.*t75.*u0.*4.0-t21.*t104.*u0.*4.0-t5.*t181.*u12.*1.2e1-t5.*t229.*u11.*4.0+t7.*t297.*u0-t11.*u44.*u45.*2.88e2-t111.*u0.*u44.*1.6e1-t185.*u0.*u43.*1.6e1+t258.*u31.*u32.*2.0+t2.*t3.*t275.*u21-t2.*t5.*t303.*u0-t5.*t6.*t333.*u21+t2.*t5.*u0.*u41.*6.0+t3.*t4.*u11.*u31.*1.92e2-t2.*t12.*u12.*u32.*1.2e1-t12.*t33.*u11.*u12.*1.08e2+t5.*t6.*u21.*u41.*4.0-t3.*t33.*u22.*u23.*2.56e2-t6.*t59.*u0.*u21.*4.0-t6.*t64.*u0.*u22.*1.2e1-t5.*t204.*u11.*u12.*1.08e2-t3.*t204.*u21.*u22.*2.56e2-t3.*t253.*u11.*u31.*1.92e2-t70.*u0.*u31.*u32.*8.0+t229.*u0.*u11.*u21.*1.8e1+t5.*t272.*u11.*u21.*u31-t6.*t300.*u0.*u11.*u31+t2.*u0.*u12.*u23.*u32.*5.4e1+t6.*u0.*u11.*u31.*u41.*8.0e1-t31.*u0.*u22.*u23.*u42.*6.4e1,-t288-t293-t344-t345+t434+t435+t436+t437+t438+t439+t440+t441+t442+t443+t444+t445+t446+t447+t448+t449+t450+t451+t452+t453+t454+t455+t456+t457+t59.*t114+t64.*t192+t21.*t258-t33.*t268.*1.28e2+t33.*t282.*1.44e2-t33.*t285.*1.92e2+t118.*t204.*1.44e2-t131.*t195.*1.44e2-t124.*t204.*1.92e2-t11.*t319.*1.44e2+t11.*t320.*1.44e2+t107.*t229.*1.8e1-t17.*t320.*1.92e2+t149.*t195.*1.44e2-t149.*t199.*1.92e2-t33.*t318.*2.7e1+t181.*t188.*1.8e1+t2.*t379+t132.*t272+t139.*t275+t156.*t297-t157.*t300-t125.*t333-t164.*t303-t126.*u45.*1.8e1-t128.*u45.*1.44e2-t205.*u44.*1.8e1-t206.*u44.*1.44e2+t209.*u44.*1.8e1+t221.*u44.*1.44e2+t224.*u44.*1.6e1-t234.*u44.*8.0e1-t246.*u44.*6.0-t249.*u44.*4.0-t286.*u43.*4.0+t290.*u41.*8.0e1-t289.*u43.*1.44e2-t294.*u43.*1.8e1+t304.*u41.*6.0+t313.*u43.*1.8e1+t314.*u43.*1.44e2+t326.*u43.*1.6e1-t327.*u43.*8.0e1+t329.*u41.*4.0-t328.*u43.*6.0+t348.*u42.*1.6e1-t351.*u42.*8.0e1-t353.*u42.*4.0+t359.*u42.*1.8e1+t363.*u42.*1.44e2-t368.*u42.*6.0-t3.*(t25.*u34+t181.*u33+t229.*u32+t338.*u31).*2.7e1+t3.*u0.*(t82.*u44+t100.*u41+t121.*u43+t253.*u42+u41.*u42.*u45.*2.0).*2.56e2-t5.*t12.*t204.*1.62e2-t3.*t28.*t204.*1.28e2-t12.*t25.*u12.*4.0-t21.*t70.*u0.*4.0-t64.*t104.*u0.*4.0-t12.*t181.*u11.*1.2e1-t5.*t229.*u12.*1.2e1-t5.*t338.*u11.*4.0+t8.*u41.*u42.*5.4e1-t111.*u0.*u45.*1.6e1-t118.*u44.*u45.*2.88e2-t185.*u0.*u44.*1.6e1-t339.*u0.*u42.*1.6e1+t343.*u31.*u32.*2.0-t2.*t31.*u0.*u22.*1.2e1+t5.*t6.*u33.*u34.*2.0-t6.*t59.*u0.*u22.*1.2e1+t3.*t6.*u41.*u42.*2.56e2-t12.*t100.*u0.*u23.*1.44e2-t12.*t149.*u11.*u12.*1.08e2-t3.*t127.*u23.*u42.*1.44e2-t3.*t149.*u22.*u23.*2.56e2-t5.*t320.*u11.*u12.*1.08e2-t3.*t320.*u21.*u22.*2.56e2+t25.*u0.*u12.*u23.*1.8e1-t75.*u0.*u31.*u32.*8.0+t338.*u0.*u11.*u21.*1.8e1-t6.*u0.*u21.*u33.*u34.*8.0-t31.*u0.*u22.*u23.*u43.*6.4e1-t12.*u12.*u23.*u34.*u42.*1.8e1+t3.*u11.*u31.*u41.*u42.*3.84e2,-t355-t357-t364-t365-t366-t374-t375-t376-t380-t381+t458+t459+t460+t461+t462+t463+t464+t465+t466+t467+t468+t469+t470+t471+t472+t473+t474+t475+t8.*t82.*2.7e1+t11.*t100.*1.44e2+t59.*t192+t64.*t258+t21.*t343+t195.*t204.*1.44e2-t199.*t204.*1.92e2-t149.*t268.*1.28e2+t188.*t229.*1.8e1+t149.*t282.*1.44e2-t149.*t285.*1.92e2-t118.*t319.*1.44e2+t118.*t320.*1.44e2-t124.*t320.*1.92e2+t107.*t338.*1.8e1-t149.*t318.*2.7e1+t209.*t272+t221.*t275+t224.*t297-t234.*t300-t246.*t303-t249.*t333+t126.*u42.*1.8e1+t128.*u42.*1.44e2-t205.*u45.*1.8e1-t206.*u45.*1.44e2-t286.*u44.*4.0-t289.*u44.*1.44e2-t294.*u44.*1.8e1+t313.*u44.*1.8e1+t314.*u44.*1.44e2+t326.*u44.*1.6e1-t327.*u44.*8.0e1-t328.*u44.*6.0+t349.*u41.*4.0+t348.*u43.*1.6e1-t351.*u43.*8.0e1-t353.*u43.*4.0+t360.*u41.*8.0e1+t359.*u43.*1.8e1+t361.*u41.*6.0+t363.*u43.*1.44e2-t368.*u43.*6.0-t3.*(t181.*u34+t229.*u33+t338.*u32+t373.*u31).*2.7e1+t3.*u0.*(t82.*u45+t100.*u42+t121.*u44+t131.*u41+t253.*u43).*2.56e2+t2.*t12.*t31+t3.*t6.*t82.*1.28e2+t5.*t6.*t127-t5.*t12.*t320.*1.62e2-t3.*t28.*t320.*1.28e2-t21.*t75.*u0.*4.0-t64.*t70.*u0.*4.0-t59.*t104.*u0.*4.0-t12.*t181.*u12.*4.0-t12.*t229.*u11.*1.2e1-t5.*t338.*u12.*1.2e1-t5.*t373.*u11.*4.0+t111.*u0.*u42.*1.6e1+t114.*u33.*u34.*2.0-t185.*u0.*u45.*1.6e1-t195.*u44.*u45.*2.88e2-t339.*u0.*u43.*1.6e1+t379.*u31.*u32.*2.0-t2.*t31.*u0.*u23.*4.0+t3.*t82.*u11.*u31.*1.92e2-t6.*t127.*u0.*u21.*4.0-t12.*t131.*u0.*u23.*1.44e2-t3.*t127.*u23.*u43.*1.44e2-t12.*t204.*u11.*u12.*1.08e2-t3.*t204.*u22.*u23.*2.56e2+t181.*u0.*u12.*u23.*1.8e1+t373.*u0.*u11.*u21.*1.8e1-t6.*u0.*u22.*u33.*u34.*2.4e1-t31.*u0.*u22.*u31.*u32.*2.4e1-t31.*u0.*u22.*u23.*u44.*6.4e1-t12.*u12.*u23.*u34.*u43.*1.8e1,-t382-t383-t384-t386-t387-t389-t391-t393-t395-t400-t401-t402-t403-t404-t405-t406+t476+t477+t478+t479+t480+t481+t482+t483+t484+t485+t486+t487+t8.*t121.*2.7e1+t11.*t131.*1.44e2+t100.*t118.*1.44e2+t114.*t127+t59.*t258+t21.*t379+t64.*t343-t204.*t268.*1.28e2+t107.*t373.*1.8e1+t204.*t282.*1.44e2-t204.*t285.*1.92e2-t195.*t319.*1.44e2+t195.*t320.*1.44e2-t199.*t320.*1.92e2-t204.*t318.*2.7e1+t188.*t338.*1.8e1+t272.*t313+t275.*t314-t286.*t333+t297.*t326-t300.*t327-t303.*t328+t126.*u43.*1.8e1+t128.*u43.*1.44e2+t205.*u42.*1.8e1+t206.*u42.*1.44e2-t289.*u45.*1.44e2-t294.*u45.*1.8e1+t348.*u44.*1.6e1-t351.*u44.*8.0e1-t353.*u44.*4.0+t359.*u44.*1.8e1+t363.*u44.*1.44e2-t368.*u44.*6.0+t390.*u41.*6.0+t392.*u41.*4.0+t394.*u41.*8.0e1-t3.*(t229.*u34+t338.*u33+t373.*u32+t399.*u31).*2.7e1+t3.*u0.*(t100.*u43+t121.*u45+t131.*u42+t253.*u44+u41.*u44.*u45.*2.0).*2.56e2+t3.*t6.*t121.*1.28e2-t59.*t70.*u0.*4.0-t64.*t75.*u0.*4.0-t12.*t229.*u12.*4.0-t12.*t338.*u11.*1.2e1-t5.*t373.*u12.*1.2e1-t5.*t399.*u11.*4.0+t111.*u0.*u43.*1.6e1+t185.*u0.*u42.*1.6e1+t192.*u33.*u34.*2.0-t339.*u0.*u44.*1.6e1-t21.*t31.*u0.*u22.*1.2e1+t12.*t31.*u31.*u32.*2.0-t6.*t127.*u0.*u22.*1.2e1+t3.*t121.*u11.*u31.*1.92e2-t3.*t127.*u23.*u44.*1.44e2-t12.*t320.*u11.*u12.*1.08e2-t3.*t320.*u22.*u23.*2.56e2-t104.*u0.*u33.*u34.*8.0+t229.*u0.*u12.*u23.*1.8e1+t399.*u0.*u11.*u21.*1.8e1-t31.*u0.*u23.*u31.*u32.*8.0-t31.*u0.*u22.*u23.*u45.*6.4e1-t12.*u0.*u23.*u44.*u45.*2.88e2-t12.*u12.*u23.*u34.*u44.*1.8e1,-t407-t409-t411-t412-t414-t416-t417-t418-t420-t421-t422-t423-t424-t425-t426-t427-t428-t429-t430-t431-t432-t433+t488+t489+t490+t491+t492+t493+t118.*t131.*1.44e2+t8.*t253.*2.7e1+t100.*t195.*1.44e2+t127.*t192+t59.*t343+t64.*t379+t107.*t399.*1.8e1+t188.*t373.*1.8e1-t268.*t320.*1.28e2+t282.*t320.*1.44e2-t285.*t320.*1.92e2+t272.*t359+t275.*t363-t318.*t320.*2.7e1+t297.*t348-t300.*t351-t303.*t368-t333.*t353+t126.*u44.*1.8e1+t128.*u44.*1.44e2+t205.*u43.*1.8e1+t206.*u43.*1.44e2+t289.*u42.*1.44e2+t294.*u42.*1.8e1+t410.*u41.*6.0+t415.*u41.*4.0+t419.*u41.*8.0e1-t3.*(t338.*u34+t373.*u33+t399.*u32+t127.*u31.*u33.*3.0).*2.7e1+t3.*u0.*(t100.*u44+t131.*u43+t253.*u45+t319.*u41+u42.*u44.*u45.*2.0).*2.56e2+t12.*t21.*t31+t3.*t6.*t253.*1.28e2-t59.*t75.*u0.*4.0-t104.*t127.*u0.*4.0-t12.*t338.*u12.*4.0-t12.*t373.*u11.*1.2e1-t5.*t399.*u12.*1.2e1+t11.*u44.*u45.*2.88e2+t111.*u0.*u44.*1.6e1+t185.*u0.*u43.*1.6e1+t258.*u33.*u34.*2.0-t339.*u0.*u45.*1.6e1-t21.*t31.*u0.*u23.*4.0-t31.*t64.*u0.*u22.*1.2e1-t5.*t127.*u11.*u33.*1.2e1-t3.*t127.*u23.*u45.*1.44e2+t3.*t253.*u11.*u31.*1.92e2-t12.*t319.*u0.*u23.*1.44e2-t70.*u0.*u33.*u34.*8.0+t338.*u0.*u12.*u23.*1.8e1+t31.*u0.*u22.*u23.*u42.*6.4e1-t12.*u12.*u23.*u34.*u45.*1.8e1+t127.*u0.*u11.*u21.*u33.*5.4e1,-t434-t435-t436-t437-t438-t439-t440-t441-t442-t443-t444-t445-t446-t447-t448-t449-t450-t451-t452-t453-t454-t455-t456-t457-t3.*(t373.*u34+t399.*u33+t127.*u31.*u34+t127.*u32.*u33.*3.0).*2.7e1+t131.*t195.*1.44e2+t11.*t319.*1.44e2+t127.*t258+t59.*t379+t188.*t399.*1.8e1+t126.*u45.*1.8e1+t128.*u45.*1.44e2+t205.*u44.*1.8e1+t206.*u44.*1.44e2+t289.*u43.*1.44e2+t294.*u43.*1.8e1+t3.*u0.*(t100.*u45+t131.*u44+t319.*u42+u43.*u44.*u45.*2.0).*2.56e2+t12.*t31.*t64-t70.*t127.*u0.*4.0+t107.*t127.*u33.*5.4e1-t12.*t373.*u12.*4.0-t12.*t399.*u11.*1.2e1+t111.*u0.*u45.*1.6e1+t118.*u44.*u45.*2.88e2+t185.*u0.*u44.*1.6e1+t339.*u0.*u42.*1.6e1+t343.*u33.*u34.*2.0-t31.*t59.*u0.*u22.*1.2e1-t31.*t64.*u0.*u23.*4.0+t12.*t100.*u0.*u23.*1.44e2-t5.*t127.*u11.*u34.*4.0-t5.*t127.*u12.*u33.*3.6e1+t3.*t127.*u23.*u42.*1.44e2-t75.*u0.*u33.*u34.*8.0+t373.*u0.*u12.*u23.*1.8e1+t31.*u0.*u22.*u23.*u43.*6.4e1+t12.*u12.*u23.*u34.*u42.*1.8e1+t127.*u0.*u11.*u21.*u34.*1.8e1,-t458-t459-t460-t461-t462-t463-t464-t465-t466-t467-t468-t469-t470-t471-t472-t473-t474-t475+t118.*t319.*1.44e2+t127.*t343+t205.*u45.*1.8e1+t206.*u45.*1.44e2+t289.*u44.*1.44e2+t294.*u44.*1.8e1-t3.*(t58.*t127.*3.0+t399.*u34+t127.*u32.*u34).*2.7e1+t3.*u0.*(t130.*u45.*2.0+t131.*u45+t319.*u43).*2.56e2+t12.*t31.*t59-t75.*t127.*u0.*4.0+t107.*t127.*u34.*1.8e1+t127.*t188.*u33.*5.4e1-t12.*t399.*u12.*4.0+t185.*u0.*u45.*1.6e1+t195.*u44.*u45.*2.88e2+t339.*u0.*u43.*1.6e1+t379.*u33.*u34.*2.0-t31.*t59.*u0.*u23.*4.0+t12.*t131.*u0.*u23.*1.44e2-t5.*t127.*u12.*u34.*1.2e1-t12.*t127.*u11.*u33.*3.6e1+t3.*t127.*u23.*u43.*1.44e2+t399.*u0.*u12.*u23.*1.8e1+t31.*u0.*u22.*u23.*u44.*6.4e1-t31.*u0.*u22.*u33.*u34.*2.4e1+t12.*u12.*u23.*u34.*u43.*1.8e1,-t476-t477-t478-t479-t480-t481-t482-t483-t484-t485-t486-t487+t127.*t379+t195.*t319.*1.44e2+t289.*u45.*1.44e2+t294.*u45.*1.8e1+t127.*t188.*u34.*1.8e1+t339.*u0.*u44.*1.6e1+t12.*t31.*u33.*u34.*2.0-t31.*t127.*u0.*u22.*1.2e1-t12.*t127.*u11.*u34.*1.2e1-t12.*t127.*u12.*u33.*1.2e1+t3.*t127.*u23.*u44.*1.44e2-t3.*t127.*u33.*u34.*1.08e2+t3.*t319.*u0.*u44.*7.68e2+t31.*u0.*u22.*u23.*u45.*6.4e1-t31.*u0.*u23.*u33.*u34.*8.0+t12.*u0.*u23.*u44.*u45.*2.88e2+t12.*u12.*u23.*u34.*u44.*1.8e1+t127.*u0.*u12.*u23.*u33.*5.4e1,-t488-t489-t490-t491-t492-t493-t3.*t127.^2.*2.7e1+t12.*t31.*t127+t339.*u0.*u45.*1.6e1-t31.*t127.*u0.*u23.*4.0-t12.*t127.*u12.*u34.*4.0+t3.*t127.*u23.*u45.*1.44e2+t12.*t319.*u0.*u23.*1.44e2+t3.*t319.*u0.*u45.*2.56e2+t12.*u12.*u23.*u34.*u45.*1.8e1+t127.*u0.*u12.*u23.*u34.*1.8e1];

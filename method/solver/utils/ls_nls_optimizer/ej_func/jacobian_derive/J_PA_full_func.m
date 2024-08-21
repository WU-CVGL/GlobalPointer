% This function is part of the GlobalPointer method as described in [1]. When you
% use this code, you are required to cite [1].
% 
% [1] GlobalPointer: Large-Scale Plane Adjustment with Bi-Convex Relaxation
% Author: B. Liao, Z. Zhao, L. Chen, H. Li, D. Cremers, P. Liu.
% European Conference on Computer Vision 2024 (ECCV 2024)
%
% 
% Author & Copyright (C) 2024: Bangyan Liao (liaobangyan[at]westlake[dot]edu[dot]cn)
%                              Zhenjun Zhao (ericzzj89[at]gmail[dot]com)
%                              Peidong Liu (liupeidong[at]westlake[dot]edu[dot]cn)

function J_full_func = J_PA_full_func(B_sqrt11,B_sqrt12,B_sqrt13,B_sqrt14,B_sqrt21,B_sqrt22,B_sqrt23,B_sqrt24,B_sqrt31,B_sqrt32,B_sqrt33,B_sqrt34,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44,phi,rx,ry,rz,theta,tx,ty,tz)

    t2 = conj(phi);
    t3 = conj(theta);
    t8 = rx.^2;
    t9 = ry.^2;
    t10 = rz.^2;
    t4 = cos(t2);
    t5 = cos(t3);
    t6 = sin(t2);
    t7 = sin(t3);
    t14 = t8+t9;
    t15 = t8+t10;
    t16 = t9+t10;
    t11 = t5.*ty;
    t12 = t6.*tz;
    t13 = t7.*tx;
    t19 = t4.*t5.*tx;
    t20 = t4.*t7.*ty;
    t21 = t10+t14;
    t17 = -t12;
    t18 = -t13;
    t22 = 1.0./t21;
    t24 = sqrt(t21);
    t23 = t11+t18;
    t25 = 1.0./t24;
    t26 = cos(t24);
    t27 = sin(t24);
    t29 = t17+t19+t20;
    t28 = t26-1.0;
    t30 = rx.*t25.*t27;
    t31 = ry.*t25.*t27;
    t32 = rz.*t25.*t27;
    t33 = rx.*ry.*t22.*t28;
    t34 = rx.*rz.*t22.*t28;
    t35 = ry.*rz.*t22.*t28;
    t39 = t14.*t22.*t28;
    t40 = t15.*t22.*t28;
    t41 = t16.*t22.*t28;
    t36 = -t33;
    t37 = -t34;
    t38 = -t35;
    t42 = t39+1.0;
    t43 = t40+1.0;
    t44 = t41+1.0;
    t57 = t30+t35;
    t58 = t31+t34;
    t59 = t32+t33;
    t45 = t4.*t43;
    t46 = t4.*t44;
    t47 = t6.*t42;
    t51 = t5.*t6.*t43;
    t53 = t6.*t7.*t44;
    t60 = t30+t38;
    t61 = t31+t37;
    t62 = t32+t36;
    t63 = t4.*t57;
    t64 = t4.*t59;
    t65 = t6.*t58;
    t71 = t5.*t6.*t57;
    t74 = t6.*t7.*t59;
    t48 = t5.*t46;
    t49 = t5.*t47;
    t50 = t7.*t45;
    t52 = t7.*t47;
    t66 = t4.*t61;
    t67 = t4.*t62;
    t68 = t6.*t60;
    t69 = t5.*t64;
    t70 = t7.*t63;
    t72 = t5.*t65;
    t73 = t7.*t65;
    t78 = t5.*t6.*t62;
    t80 = t6.*t7.*t61;
    t85 = t51+t74;
    t55 = -t50;
    t75 = t5.*t66;
    t76 = t7.*t67;
    t77 = t5.*t68;
    t79 = t7.*t68;
    t83 = t46+t72;
    t84 = t52+t63;
    t88 = B_sqrt21.*t85;
    t89 = B_sqrt22.*t85;
    t90 = B_sqrt23.*t85;
    t91 = B_sqrt24.*t85;
    t93 = -B_sqrt11.*(t53-t78);
    t94 = -B_sqrt12.*(t53-t78);
    t95 = -B_sqrt13.*(t53-t78);
    t96 = -B_sqrt14.*(t53-t78);
    t98 = t67+t73;
    t99 = t71+t80;
    t81 = -t75;
    t82 = -t79;
    t97 = t64+t77;
    t100 = B_sqrt31.*t99;
    t101 = B_sqrt32.*t99;
    t102 = B_sqrt33.*t99;
    t103 = B_sqrt34.*t99;
    t104 = t48+t65+t76;
    t106 = t55+t68+t69;
    t87 = t45+t82;
    t105 = t47+t70+t81;
    mt1 = [B_sqrt21.*t87+B_sqrt11.*t98-B_sqrt31.*t84,B_sqrt22.*t87+B_sqrt12.*t98-B_sqrt32.*t84,B_sqrt23.*t87+B_sqrt13.*t98-B_sqrt33.*t84,B_sqrt24.*t87+B_sqrt14.*t98-B_sqrt34.*t84,-B_sqrt11.*t83+B_sqrt21.*t97+B_sqrt31.*(t49-t66),-B_sqrt12.*t83+B_sqrt22.*t97+B_sqrt32.*(t49-t66),-B_sqrt13.*t83+B_sqrt23.*t97+B_sqrt33.*(t49-t66),-B_sqrt14.*t83+B_sqrt24.*t97+B_sqrt34.*(t49-t66),-t88+t100+B_sqrt11.*(t53-t78),-t89+t101+B_sqrt12.*(t53-t78),-t90+t102+B_sqrt13.*(t53-t78),-t91+t103+B_sqrt14.*(t53-t78),B_sqrt41.*t5.*t6,B_sqrt42.*t5.*t6,B_sqrt43.*t5.*t6,B_sqrt44.*t5.*t6,B_sqrt41.*t6.*t7,B_sqrt42.*t6.*t7,B_sqrt43.*t6.*t7];
    mt2 = [B_sqrt44.*t6.*t7,B_sqrt41.*t4,B_sqrt42.*t4,B_sqrt43.*t4,B_sqrt44.*t4,B_sqrt41.*t29+B_sqrt11.*t104-B_sqrt21.*t106-B_sqrt31.*t105,B_sqrt42.*t29+B_sqrt12.*t104-B_sqrt22.*t106-B_sqrt32.*t105,B_sqrt43.*t29+B_sqrt13.*t104-B_sqrt23.*t106-B_sqrt33.*t105,B_sqrt44.*t29+B_sqrt14.*t104-B_sqrt24.*t106-B_sqrt34.*t105,t88+t93-t100+B_sqrt41.*t6.*t23,t89+t94-t101+B_sqrt42.*t6.*t23,t90+t95-t102+B_sqrt43.*t6.*t23,t91+t96-t103+B_sqrt44.*t6.*t23,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44];
    J_full_func = reshape([mt1,mt2],4,9);

end

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

function out1 = J_PA_pose_func(B_sqrt11,B_sqrt12,B_sqrt13,B_sqrt14,B_sqrt21,B_sqrt22,B_sqrt23,B_sqrt24,B_sqrt31,B_sqrt32,B_sqrt33,B_sqrt34,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44,phi,rx,ry,rz,theta)

    t2 = conj(phi);
    t3 = conj(theta);
    t8 = rx.^2;
    t9 = ry.^2;
    t10 = rz.^2;
    t4 = cos(t2);
    t5 = cos(t3);
    t6 = sin(t2);
    t7 = sin(t3);
    t11 = t8+t9;
    t12 = t8+t10;
    t13 = t9+t10;
    t14 = t10+t11;
    t15 = 1.0./t14;
    t16 = sqrt(t14);
    t17 = 1.0./t16;
    t18 = cos(t16);
    t19 = sin(t16);
    t20 = t18-1.0;
    t21 = rx.*t17.*t19;
    t22 = ry.*t17.*t19;
    t23 = rz.*t17.*t19;
    t24 = rx.*ry.*t15.*t20;
    t25 = rx.*rz.*t15.*t20;
    t26 = ry.*rz.*t15.*t20;
    t30 = t11.*t15.*t20;
    t31 = t12.*t15.*t20;
    t32 = t13.*t15.*t20;
    t27 = -t24;
    t28 = -t25;
    t29 = -t26;
    t33 = t30+1.0;
    t34 = t31+1.0;
    t35 = t32+1.0;
    t44 = t21+t26;
    t45 = t22+t25;
    t46 = t23+t24;
    t36 = t4.*t34;
    t37 = t4.*t35;
    t38 = t5.*t6.*t33;
    t39 = t5.*t6.*t34;
    t40 = t6.*t7.*t33;
    t41 = t6.*t7.*t35;
    t47 = t21+t29;
    t48 = t22+t28;
    t49 = t23+t27;
    t50 = t4.*t44;
    t51 = t4.*t46;
    t54 = t5.*t6.*t44;
    t55 = t5.*t6.*t45;
    t56 = t6.*t7.*t45;
    t57 = t6.*t7.*t46;
    t52 = t4.*t48;
    t53 = t4.*t49;
    t58 = t5.*t6.*t47;
    t59 = t5.*t6.*t49;
    t60 = t6.*t7.*t47;
    t61 = t6.*t7.*t48;
    t63 = t37+t55;
    t64 = t40+t50;
    t65 = t39+t57;
    t62 = -t60;
    t69 = t51+t58;
    t70 = t53+t56;
    t71 = t54+t61;
    t67 = t36+t62;
    mt1 = [B_sqrt11.*t70+B_sqrt21.*t67-B_sqrt31.*t64,B_sqrt12.*t70+B_sqrt22.*t67-B_sqrt32.*t64,B_sqrt13.*t70+B_sqrt23.*t67-B_sqrt33.*t64,B_sqrt14.*t70+B_sqrt24.*t67-B_sqrt34.*t64,-B_sqrt11.*t63+B_sqrt21.*t69+B_sqrt31.*(t38-t52),-B_sqrt12.*t63+B_sqrt22.*t69+B_sqrt32.*(t38-t52),-B_sqrt13.*t63+B_sqrt23.*t69+B_sqrt33.*(t38-t52),-B_sqrt14.*t63+B_sqrt24.*t69+B_sqrt34.*(t38-t52),-B_sqrt21.*t65+B_sqrt31.*t71+B_sqrt11.*(t41-t59),-B_sqrt22.*t65+B_sqrt32.*t71+B_sqrt12.*(t41-t59),-B_sqrt23.*t65+B_sqrt33.*t71+B_sqrt13.*(t41-t59),-B_sqrt24.*t65+B_sqrt34.*t71+B_sqrt14.*(t41-t59),B_sqrt41.*t5.*t6,B_sqrt42.*t5.*t6,B_sqrt43.*t5.*t6,B_sqrt44.*t5.*t6];
    mt2 = [B_sqrt41.*t6.*t7,B_sqrt42.*t6.*t7,B_sqrt43.*t6.*t7,B_sqrt44.*t6.*t7,B_sqrt41.*t4,B_sqrt42.*t4,B_sqrt43.*t4,B_sqrt44.*t4];
    out1 = reshape([mt1,mt2],4,6);

end

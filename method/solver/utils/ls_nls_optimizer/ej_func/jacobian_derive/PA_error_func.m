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

function error = PA_error_func(B_sqrt11,B_sqrt12,B_sqrt13,B_sqrt14,B_sqrt21,B_sqrt22,B_sqrt23,B_sqrt24,B_sqrt31,B_sqrt32,B_sqrt33,B_sqrt34,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44,nq,phi,rx,ry,rz,theta,tx,ty,tz)

    t2 = conj(nq);
    t3 = conj(phi);
    t4 = conj(theta);
    t9 = rx.^2;
    t10 = ry.^2;
    t11 = rz.^2;
    t5 = cos(t3);
    t6 = cos(t4);
    t7 = sin(t3);
    t8 = sin(t4);
    t13 = t9+t10;
    t14 = t9+t11;
    t15 = t10+t11;
    t12 = t5.*tz;
    t16 = t6.*t7.*tx;
    t17 = t7.*t8.*ty;
    t18 = t11+t13;
    t19 = 1.0./t18;
    t20 = sqrt(t18);
    t25 = t2+t12+t16+t17;
    t21 = 1.0./t20;
    t22 = cos(t20);
    t23 = sin(t20);
    t24 = t22-1.0;
    t26 = rx.*t21.*t23;
    t27 = ry.*t21.*t23;
    t28 = rz.*t21.*t23;
    t29 = rx.*ry.*t19.*t24;
    t30 = rx.*rz.*t19.*t24;
    t31 = ry.*rz.*t19.*t24;
    t35 = t13.*t19.*t24;
    t36 = t14.*t19.*t24;
    t37 = t15.*t19.*t24;
    t32 = -t29;
    t33 = -t30;
    t34 = -t31;
    t38 = t35+1.0;
    t39 = t36+1.0;
    t40 = t37+1.0;
    t44 = t26+t31;
    t45 = t27+t30;
    t46 = t28+t29;
    t41 = t5.*t38;
    t42 = t6.*t7.*t40;
    t43 = t7.*t8.*t39;
    t47 = t26+t34;
    t48 = t27+t33;
    t49 = t28+t32;
    t50 = t5.*t45;
    t53 = t6.*t7.*t46;
    t54 = t7.*t8.*t44;
    t51 = t5.*t47;
    t52 = -t50;
    t55 = t6.*t7.*t48;
    t56 = -t53;
    t57 = t7.*t8.*t49;
    t58 = -t54;
    t59 = t41+t55+t58;
    t60 = t43+t51+t56;
    t61 = t42+t52+t57;
    error = [B_sqrt41.*t25+B_sqrt11.*t61+B_sqrt21.*t60+B_sqrt31.*t59,B_sqrt42.*t25+B_sqrt12.*t61+B_sqrt22.*t60+B_sqrt32.*t59,B_sqrt43.*t25+B_sqrt13.*t61+B_sqrt23.*t60+B_sqrt33.*t59,B_sqrt44.*t25+B_sqrt14.*t61+B_sqrt24.*t60+B_sqrt34.*t59];

end

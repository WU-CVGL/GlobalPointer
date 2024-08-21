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

function out1 = J_PA_plane_func(B_sqrt11,B_sqrt12,B_sqrt13,B_sqrt14,B_sqrt21,B_sqrt22,B_sqrt23,B_sqrt24,B_sqrt31,B_sqrt32,B_sqrt33,B_sqrt34,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44,phi,rx,ry,rz,theta,tx,ty,tz)

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
    t52 = t30+t35;
    t53 = t31+t34;
    t54 = t32+t33;
    t45 = t6.*t42;
    t46 = t4.*t5.*t44;
    t47 = t4.*t7.*t43;
    t48 = t5.*t6.*t43;
    t49 = t6.*t7.*t44;
    t55 = t30+t38;
    t56 = t31+t37;
    t57 = t32+t36;
    t58 = t6.*t53;
    t60 = t4.*t5.*t54;
    t61 = t4.*t7.*t52;
    t62 = t5.*t6.*t52;
    t63 = t6.*t7.*t54;
    t50 = -t47;
    t59 = t6.*t55;
    t64 = t4.*t5.*t56;
    t65 = t4.*t7.*t57;
    t66 = t5.*t6.*t57;
    t67 = t6.*t7.*t56;
    t69 = t48+t63;
    t68 = -t64;
    t71 = t62+t67;
    t72 = t46+t58+t65;
    t74 = t50+t59+t60;
    t73 = t45+t61+t68;
    out1 = reshape([B_sqrt41.*t29+B_sqrt11.*t72-B_sqrt21.*t74-B_sqrt31.*t73,B_sqrt42.*t29+B_sqrt12.*t72-B_sqrt22.*t74-B_sqrt32.*t73,B_sqrt43.*t29+B_sqrt13.*t72-B_sqrt23.*t74-B_sqrt33.*t73,B_sqrt44.*t29+B_sqrt14.*t72-B_sqrt24.*t74-B_sqrt34.*t73,B_sqrt21.*t69-B_sqrt31.*t71-B_sqrt11.*(t49-t66)+B_sqrt41.*t6.*t23,B_sqrt22.*t69-B_sqrt32.*t71-B_sqrt12.*(t49-t66)+B_sqrt42.*t6.*t23,B_sqrt23.*t69-B_sqrt33.*t71-B_sqrt13.*(t49-t66)+B_sqrt43.*t6.*t23,B_sqrt24.*t69-B_sqrt34.*t71-B_sqrt14.*(t49-t66)+B_sqrt44.*t6.*t23,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44],[4,3]);

end

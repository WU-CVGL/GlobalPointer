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

function J_full_func = J_full_func(P1,P2,P3,nx,ny,nz,rx,ry,rz,tx,ty,tz)

    t2 = rx.^2;
    t3 = ry.^2;
    t4 = rz.^2;
    t5 = t2+t3;
    t6 = t2+t4;
    t7 = t3+t4;
    t8 = t4+t5;
    t9 = 1.0./t8;
    t11 = sqrt(t8);
    t10 = t9.^2;
    t12 = 1.0./t11;
    t14 = cos(t11);
    t15 = sin(t11);
    t13 = t12.^3;
    t16 = t14-1.0;
    t17 = rx.*ry.*t9.*t14;
    t18 = rx.*rz.*t9.*t14;
    t19 = ry.*rz.*t9.*t14;
    t20 = t12.*t15;
    t24 = t2.*t9.*t14;
    t25 = t3.*t9.*t14;
    t26 = t4.*t9.*t14;
    t21 = rx.*t20;
    t22 = ry.*t20;
    t23 = rz.*t20;
    t27 = -t17;
    t28 = -t18;
    t29 = -t19;
    t30 = rx.*ry.*t13.*t15;
    t31 = rx.*rz.*t13.*t15;
    t32 = ry.*rz.*t13.*t15;
    t33 = -t20;
    t34 = rx.*t9.*t16;
    t35 = ry.*t9.*t16;
    t36 = rz.*t9.*t16;
    t37 = t2.*t13.*t15;
    t38 = t3.*t13.*t15;
    t39 = t4.*t13.*t15;
    t62 = rx.*ry.*rz.*t10.*t16.*2.0;
    t63 = rx.*t3.*t10.*t16.*2.0;
    t64 = ry.*t2.*t10.*t16.*2.0;
    t65 = rx.*t4.*t10.*t16.*2.0;
    t66 = rz.*t2.*t10.*t16.*2.0;
    t67 = ry.*t4.*t10.*t16.*2.0;
    t68 = rz.*t3.*t10.*t16.*2.0;
    t40 = ry.*t34;
    t41 = rz.*t34;
    t42 = rz.*t35;
    t43 = rz.*t30;
    t44 = t34.*2.0;
    t45 = t35.*2.0;
    t46 = t36.*2.0;
    t47 = -t30;
    t48 = -t31;
    t49 = -t32;
    t50 = rx.*t38;
    t51 = ry.*t37;
    t52 = rx.*t39;
    t53 = rz.*t37;
    t54 = ry.*t39;
    t55 = rz.*t38;
    t56 = -t34;
    t58 = -t35;
    t60 = -t36;
    t57 = -t44;
    t59 = -t45;
    t61 = -t46;
    mt1 = ny.*(P1.*(t18+t48+t51+t58+t64)+P3.*(-t24+t33+t37+t43+t62)-P2.*(t57+rx.*t6.*t10.*t16.*2.0+rx.*t6.*t13.*t15))+nz.*(P1.*(t27+t30+t53+t60+t66)+P2.*(t20+t24-t37+t43+t62)-P3.*(t57+rx.*t5.*t10.*t16.*2.0+rx.*t5.*t13.*t15))+nx.*(P2.*(t28+t31+t51+t58+t64)+P3.*(t17+t47+t53+t60+t66)-P1.*(rx.*t7.*t10.*t16.*2.0+rx.*t7.*t13.*t15));
    mt2 = nx.*(P2.*(t29+t32+t50+t56+t63)+P3.*(t20+t25-t38+t43+t62)-P1.*(t59+ry.*t7.*t10.*t16.*2.0+ry.*t7.*t13.*t15))+nz.*(P2.*(t17+t47+t55+t60+t68)+P1.*(-t25+t33+t38+t43+t62)-P3.*(t59+ry.*t5.*t10.*t16.*2.0+ry.*t5.*t13.*t15))+ny.*(P1.*(t19+t49+t50+t56+t63)+P3.*(t27+t30+t55+t60+t68)-P2.*(ry.*t6.*t10.*t16.*2.0+ry.*t6.*t13.*t15));
    mt3 = [nx.*(P3.*(t19+t49+t52+t56+t65)+P2.*(-t26+t33+t39+t43+t62)-P1.*(t61+rz.*t7.*t10.*t16.*2.0+rz.*t7.*t13.*t15))+ny.*(P3.*(t28+t31+t54+t58+t67)+P1.*(t20+t26-t39+t43+t62)-P2.*(t61+rz.*t6.*t10.*t16.*2.0+rz.*t6.*t13.*t15))+nz.*(P1.*(t29+t32+t52+t56+t65)+P2.*(t18+t48+t54+t58+t67)-P3.*(rz.*t5.*t10.*t16.*2.0+rz.*t5.*t13.*t15)),nx,ny,nz,tx+P3.*(t22-t41)+P1.*(t7.*t9.*t16+1.0)-P2.*(t23+t40),ty+P1.*(t23-t40)+P2.*(t6.*t9.*t16+1.0)-P3.*(t21+t42),tz+P2.*(t21-t42)+P3.*(t5.*t9.*t16+1.0)-P1.*(t22+t41),1.0];
    J_full_func = [mt1,mt2,mt3];

end

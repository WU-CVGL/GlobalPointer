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

function J_pose_func = J_pose_func(P1,P2,P3,nx,ny,nz,rx,ry,rz)

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
    t21 = t2.*t9.*t14;
    t22 = t3.*t9.*t14;
    t23 = t4.*t9.*t14;
    t24 = -t17;
    t25 = -t18;
    t26 = -t19;
    t27 = rx.*ry.*t13.*t15;
    t28 = rx.*rz.*t13.*t15;
    t29 = ry.*rz.*t13.*t15;
    t30 = -t20;
    t31 = rx.*t9.*t16;
    t32 = ry.*t9.*t16;
    t33 = rz.*t9.*t16;
    t34 = t2.*t13.*t15;
    t35 = t3.*t13.*t15;
    t36 = t4.*t13.*t15;
    t56 = rx.*ry.*rz.*t10.*t16.*2.0;
    t57 = rx.*t3.*t10.*t16.*2.0;
    t58 = ry.*t2.*t10.*t16.*2.0;
    t59 = rx.*t4.*t10.*t16.*2.0;
    t60 = rz.*t2.*t10.*t16.*2.0;
    t61 = ry.*t4.*t10.*t16.*2.0;
    t62 = rz.*t3.*t10.*t16.*2.0;
    t37 = rz.*t27;
    t38 = t31.*2.0;
    t39 = t32.*2.0;
    t40 = t33.*2.0;
    t41 = -t27;
    t42 = -t28;
    t43 = -t29;
    t44 = rx.*t35;
    t45 = ry.*t34;
    t46 = rx.*t36;
    t47 = rz.*t34;
    t48 = ry.*t36;
    t49 = rz.*t35;
    t50 = -t31;
    t52 = -t32;
    t54 = -t33;
    t51 = -t38;
    t53 = -t39;
    t55 = -t40;
    mt1 = ny.*(P1.*(t18+t42+t45+t52+t58)+P3.*(-t21+t30+t34+t37+t56)-P2.*(t51+rx.*t6.*t10.*t16.*2.0+rx.*t6.*t13.*t15))+nz.*(P1.*(t24+t27+t47+t54+t60)+P2.*(t20+t21-t34+t37+t56)-P3.*(t51+rx.*t5.*t10.*t16.*2.0+rx.*t5.*t13.*t15))+nx.*(P2.*(t25+t28+t45+t52+t58)+P3.*(t17+t41+t47+t54+t60)-P1.*(rx.*t7.*t10.*t16.*2.0+rx.*t7.*t13.*t15));
    mt2 = nx.*(P2.*(t26+t29+t44+t50+t57)+P3.*(t20+t22-t35+t37+t56)-P1.*(t53+ry.*t7.*t10.*t16.*2.0+ry.*t7.*t13.*t15))+nz.*(P2.*(t17+t41+t49+t54+t62)+P1.*(-t22+t30+t35+t37+t56)-P3.*(t53+ry.*t5.*t10.*t16.*2.0+ry.*t5.*t13.*t15))+ny.*(P1.*(t19+t43+t44+t50+t57)+P3.*(t24+t27+t49+t54+t62)-P2.*(ry.*t6.*t10.*t16.*2.0+ry.*t6.*t13.*t15));
    mt3 = [nx.*(P3.*(t19+t43+t46+t50+t59)+P2.*(-t23+t30+t36+t37+t56)-P1.*(t55+rz.*t7.*t10.*t16.*2.0+rz.*t7.*t13.*t15))+ny.*(P3.*(t25+t28+t48+t52+t61)+P1.*(t20+t23-t36+t37+t56)-P2.*(t55+rz.*t6.*t10.*t16.*2.0+rz.*t6.*t13.*t15))+nz.*(P1.*(t26+t29+t46+t50+t59)+P2.*(t18+t42+t48+t52+t61)-P3.*(rz.*t5.*t10.*t16.*2.0+rz.*t5.*t13.*t15)),nx,ny,nz];
    J_pose_func = [mt1,mt2,mt3];

end

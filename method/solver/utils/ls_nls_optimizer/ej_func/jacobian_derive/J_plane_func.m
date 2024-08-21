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

function J_plane_func = J_plane_func(P1,P2,P3,rx,ry,rz,tx,ty,tz)

    t2 = rx.^2;
    t3 = ry.^2;
    t4 = rz.^2;
    t5 = t2+t3+t4;
    t6 = 1.0./t5;
    t7 = sqrt(t5);
    t8 = 1.0./t7;
    t9 = cos(t7);
    t10 = sin(t7);
    t11 = t9-1.0;
    t12 = rx.*t8.*t10;
    t13 = ry.*t8.*t10;
    t14 = rz.*t8.*t10;
    t15 = rx.*ry.*t6.*t11;
    t16 = rx.*rz.*t6.*t11;
    t17 = ry.*rz.*t6.*t11;
    J_plane_func = [tx+P3.*(t13-t16)+P1.*(t6.*t11.*(t3+t4)+1.0)-P2.*(t14+t15),ty+P1.*(t14-t15)+P2.*(t6.*t11.*(t2+t4)+1.0)-P3.*(t12+t17),tz+P2.*(t12-t17)+P3.*(t6.*t11.*(t2+t3)+1.0)-P1.*(t13+t16),1.0];

end

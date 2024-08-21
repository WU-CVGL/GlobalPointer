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

function [] = PA_JMatrix_generator()

    % ----------------- file_path -----------------
    file_path = 'PATH_TO_PROJECT\GlobalPointer\method\solver\utils\ls_nls_optimizer\ej_func\jacobian_derive\';

    % ----------------- symsbol -----------------
    syms theta phi
    syms nq 
    syms rx ry rz
    syms tx ty tz
    syms drx dry drz
    B_sqrt = sym('B_sqrt%d%d', [4 4]);

    % ----------------- interference -----------------
    % Expression for the rotation matrix based on the Rodrigues formula
    theta_r=sqrt(rx^2+ry^2+rz^2);
    omega =  [0 -rz ry;
            rz 0 -rx;
            -ry rx 0;];
    R = eye(3) + (sin(theta_r)/theta_r)*omega + ((1-cos(theta_r))/theta_r^2)*(omega*omega);
    dR = [1 -drz dry;
        drz 1 -drx;
        -dry drx 1;];
    R = dR * R;
    t = [tx;ty;tz];
    T = [R,t;0 0 0 1];

    n = [sin(phi)*cos(theta);sin(phi)*sin(theta);cos(phi);nq];

    % ----------------- error definition -----------------
    error = n' * T * B_sqrt;
    J_full_func = jacobian(error, [drx,dry,drz,tx,ty,tz,phi,theta,nq]);
    J_full_func = subs(J_full_func,drx,0);
    J_full_func = subs(J_full_func,dry,0);
    J_full_func = subs(J_full_func,drz,0);
    J_full_func = simplify(J_full_func);
    J_func = matlabFunction(J_full_func, 'File', [file_path , 'J_PA_full_func']);
    J_func = matlabFunction(J_full_func(:,1:6), 'File', [file_path , 'J_PA_pose_func']);
    J_func = matlabFunction(J_full_func(:,7:9), 'File', [file_path , 'J_PA_plane_func']);

    error = subs(error,drx,0);
    error = subs(error,dry,0);
    error = subs(error,drz,0);
    error = simplify(error);
    J_func = matlabFunction(error, 'File', [file_path , 'PA_error_func']);

end

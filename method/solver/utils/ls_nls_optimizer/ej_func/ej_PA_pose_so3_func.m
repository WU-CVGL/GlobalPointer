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

function [F,J] = ej_PA_pose_so3_func(x, plane, B_sqrt_cell)
    
    lidar_pose_num = 1;
    plane_num = size(B_sqrt_cell, 2);
    
    F_cell = cell(lidar_pose_num, plane_num);
    J_cell = cell(lidar_pose_num, plane_num);

    r_vec = x(1,1:3);
    R = axang2rotm([r_vec / norm(r_vec) norm(r_vec)]);
    t = x(1,4:6);

    for plane_i = 1:plane_num
        plane_id = (plane_i - 1) * 3;
        n_spher = plane(1, plane_id+1:plane_id+2);
        nq = plane(1, plane_id+3);
        phi = n_spher(1);
        theta = n_spher(2);

        B_sqrt = B_sqrt_cell{1, plane_i};

        T = [R, t'; 0 0 0 1];

        n = [sin(phi)*cos(theta);sin(phi)*sin(theta);cos(phi);nq];

        % ----------------- error definition -----------------
        F_single = n' * T * B_sqrt;

        B_sqrt_T = T * B_sqrt;
        J_tmp_r = (-[0 -n(3) n(2); n(3) 0 -n(1); -n(2) n(1) 0] * B_sqrt_T(1:3, :))';
        J_tmp_t = (n(1:3, 1) * B_sqrt(4, :))';

        F_cell{1, plane_i} = F_single;
        J_cell{1, plane_i} = [J_tmp_r,J_tmp_t];
    end

    F_cell = reshape(F_cell, [1, lidar_pose_num * plane_num]);
    J_cell = reshape(J_cell, [lidar_pose_num * plane_num, 1]);

    F = cell2mat(F_cell);
    J = cell2mat(J_cell);

end
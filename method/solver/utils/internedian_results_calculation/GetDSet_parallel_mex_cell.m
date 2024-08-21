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

function D_sets = GetDSet_parallel_mex_cell(B_sets, plane_estimate_struct)

    lidar_pose_num = size(B_sets, 1);
    plane_num = size(B_sets, 2);
    A_sets = cell(1, plane_num);

    for plane_i = 1:plane_num
        A_sets{1, plane_i} = plane_estimate_struct(:,plane_i) * plane_estimate_struct(:,plane_i)';
    end
    
    D_sets = cell(1, lidar_pose_num);
    for lidar_pose_i = 1:lidar_pose_num
        D = zeros(16, 16);
        for plane_i = 1:plane_num
            D = D + kron(A_sets{1, plane_i},B_sets{lidar_pose_i, plane_i});
        end
        D_sets{1, lidar_pose_i} = D;
    end

end
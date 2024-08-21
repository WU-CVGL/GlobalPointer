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

function B_sets_global = Bsets_transform_func(T_cell, B_sets)

    lidar_pose_num = size(B_sets, 1);
    plane_num = size(B_sets, 2);

    B_sets_global = B_sets;
    
    for lidar_pose_i = 1:lidar_pose_num
        T = T_cell{1, lidar_pose_i};
        for plane_i = 1:plane_num
            B_sets_global{lidar_pose_i, plane_i} = T * B_sets_global{lidar_pose_i, plane_i} * T';
        end
    end

end
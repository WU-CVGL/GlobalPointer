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

function [B_sets, inlier_laplace_matrix] = Outlier_Bset(B_sets, outlier_ratio, param)

    [num_lidar_poses, num_planes] = size(B_sets);
    inlier_laplace_matrix = ones(num_lidar_poses, num_planes);

    for lidar_pose_i = 1:param.lidar_pose_num
        outlier_rand_id = randperm(param.plane_num, param.plane_num*outlier_ratio);

        for i = 1:size(outlier_rand_id, 2)
            rand_m = randn(4) * randn(4)';
            B_sets{lidar_pose_i, outlier_rand_id(1, i)} = rand_m * norm(B_sets{lidar_pose_i, outlier_rand_id(1, i)});
            inlier_laplace_matrix(lidar_pose_i, outlier_rand_id(1, i)) = 0;
        end
    end
   
end
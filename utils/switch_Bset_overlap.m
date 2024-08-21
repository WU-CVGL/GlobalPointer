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

function [switch_B_sets, laplace_matrix] = switch_Bset_overlap(B_sets, connection_num, connection_switch_ratio, param)

    while true
        laplace_matrix = ones(size(B_sets, [1, 2]));
        switch_B_sets = B_sets;
        inlier_id = 1:connection_num;
        all_id = 1:param.plane_num;

        for lidar_pose_i = 1:param.lidar_pose_num
            outlier_id = setdiff(all_id, inlier_id);
            for plane_j = 1:param.plane_num - connection_num
                switch_B_sets{lidar_pose_i, outlier_id(1, plane_j)} = zeros(4,4);
                laplace_matrix(lidar_pose_i, outlier_id(1, plane_j)) = 0;
            end
    
            inlier_rand_id = randperm(connection_num,connection_num*connection_switch_ratio);
            inlier_rand_id = sort(inlier_rand_id);
            inlier_id = inlier_id(1, inlier_rand_id);
        
            outlier_id = setdiff(all_id, inlier_id);
            inlier_rand_id = randperm(size(outlier_id, 2),connection_num*(1-connection_switch_ratio));
            inlier_rand_id = sort(inlier_rand_id);
            inlier2_id = outlier_id(1, inlier_rand_id);
    
            inlier_id = union(inlier_id, inlier2_id);
        end
    
        if all(sum(laplace_matrix, 1) > 6)
            break;
        end
    end

end
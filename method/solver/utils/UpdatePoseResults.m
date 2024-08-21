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

function [lidar_pose_estimate_struct, lidar_pose_estimate_matrix, optimal_pose_vec] = UpdatePoseResults(Y_list, param)

    optimal_pose_vec = ones(1, param.lidar_pose_num);

    for lidar_pose_i = 1:param.lidar_pose_num
        [~, S, V] = svd(Y_list{1, lidar_pose_i});
        A = S * V';
        x_solution = A(1, :);
        x_solution = x_solution / x_solution(1, 13);

        R = [x_solution(1, 1:3); x_solution(1, 4:6); x_solution(1, 7:9)];
        t = x_solution(1, 10:12);
        R_near = SearchNearestRotation(R);
        lidar_pose_estimate_struct(lidar_pose_i).R = R_near;
        lidar_pose_estimate_struct(lidar_pose_i).t = t;
    end

    lidar_pose_estimate_matrix = cell(param.lidar_pose_num,1);

    for lidar_pose_i = 1:param.lidar_pose_num
        Tmp = zeros(4, 4);
        Tmp(1:3, 1:3) = lidar_pose_estimate_struct(lidar_pose_i).R;
        Tmp(1:3, 4) = lidar_pose_estimate_struct(lidar_pose_i).t';
        Tmp(4, 4) = 1;
        lidar_pose_estimate_matrix{lidar_pose_i} = Tmp;
    end

end
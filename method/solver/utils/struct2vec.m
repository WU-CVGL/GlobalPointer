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

function [lidar_pose_estimate_matrix, plane_estimate_matrix] = struct2vec(lidar_pose_struct, plane_struct, param)

    lidar_pose_estimate_matrix = cell(param.lidar_pose_num,1);

    for lidar_pose_i = 1:param.lidar_pose_num
        Tmp = zeros(4, 4);
        Tmp(1:3, 1:3) = lidar_pose_struct(lidar_pose_i).R;
        Tmp(1:3, 4) = lidar_pose_struct(lidar_pose_i).t';
        Tmp(4, 4) = 1;
        lidar_pose_estimate_matrix{lidar_pose_i} = Tmp;
    end

    plane_estimate_matrix = zeros(4, param.plane_num);

    for plane_i = 1:param.plane_num
        plane_estimate_matrix(:, plane_i) = [plane_struct.normal_vector_list{plane_i}; plane_struct.q_list{plane_i}];
    end

end
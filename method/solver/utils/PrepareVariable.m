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

function [lidar_pose_estimate_struct, plane_estimate_struct] = PrepareVariable(lidar_pose_struct, plane_struct, param)

    lidar_pose_estimate_struct = [];

    for lidar_pose_i = 1:param.lidar_pose_num
        lidar_pose_estimate_struct(lidar_pose_i).R = lidar_pose_struct(lidar_pose_i).R;
        lidar_pose_estimate_struct(lidar_pose_i).t = lidar_pose_struct(lidar_pose_i).t;
    end

    plane_estimate_struct.normal_vector_list = plane_struct.normal_vector_list;
    plane_estimate_struct.q_list = cell(1, param.plane_num);
    
    for i = 1:param.plane_num
        plane_estimate_struct.q_list{i} = -plane_struct.q_list{i}' * plane_struct.normal_vector_list{i};
    end

end
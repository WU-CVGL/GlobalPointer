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

function [lidar_pose_estimate_vec, plane_estimate_vec] = PrepareVariableVec(lidar_pose_struct, plane_struct, param)

    lidar_pose_estimate_vec = zeros(6, param.lidar_pose_num);

    for lidar_pose_i = 1:param.lidar_pose_num
        axang = rotm2axang(lidar_pose_struct(lidar_pose_i).R);
        r = axang(1:3) * axang(4);
        t = lidar_pose_struct(lidar_pose_i).t';
        lidar_pose_estimate_vec(:, lidar_pose_i) = [r';t];
    end

    lidar_pose_estimate_vec = reshape(lidar_pose_estimate_vec, [1, 6 * param.lidar_pose_num]);
    plane_estimate_vec = zeros(4, param.plane_num);

    for plane_i = 1:param.plane_num
        if size(plane_struct.q_list{plane_i}, 1) ~= 1
            plane_estimate_vec(:, plane_i) = [plane_struct.normal_vector_list{plane_i}; -plane_struct.q_list{plane_i}' * plane_struct.normal_vector_list{plane_i}];
        else
            plane_estimate_vec(:, plane_i) = [plane_struct.normal_vector_list{plane_i}; plane_struct.q_list{plane_i}];
        end
        
    end
    
    plane_estimate_vec = reshape(plane_estimate_vec, [1, 4 * param.plane_num]);

end
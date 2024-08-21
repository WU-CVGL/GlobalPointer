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

function [lidar_pose_error_struct, plane_error_struct] = RecoverResults(lidar_pose_estimate_struct, plane_estimate_struct, param)

    lidar_pose_error_struct = lidar_pose_estimate_struct;
    plane_error_struct = plane_estimate_struct;
    
    % ---- recover results ----
    Rotation_recover = lidar_pose_estimate_struct(1).R';
    Translation_recover = lidar_pose_estimate_struct(1).t;
    
    for plane_i = 1:param.plane_num
        plane_error_struct.normal_vector_list{1, plane_i} = Rotation_recover * plane_estimate_struct.normal_vector_list{1, plane_i};
        plane_error_struct.q_list{1, plane_i} = plane_estimate_struct.q_list{1, plane_i} + plane_estimate_struct.normal_vector_list{1, plane_i}' * Translation_recover';
    end

    for lidar_pose_i = 1:param.lidar_pose_num
        lidar_pose_error_struct(lidar_pose_i).R = Rotation_recover * lidar_pose_error_struct(lidar_pose_i).R;
        lidar_pose_error_struct(lidar_pose_i).t = (lidar_pose_error_struct(lidar_pose_i).t - Translation_recover) * Rotation_recover';
    end
    
end
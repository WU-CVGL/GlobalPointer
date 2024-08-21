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

function [plane_struct, lidar_pose_struct] = pertub_plane_and_pose(plane_struct, lidar_pose_struct, param)

    % -------- pose data generate --------    
    for lidar_pose_i = 1:param.lidar_pose_num
        if(param.randn_pose)
            noise_r = randn(1, 3);
            lidar_pose_struct(lidar_pose_i).R = axang2rotm([noise_r / norm(noise_r), norm(noise_r)]);
            lidar_pose_struct(lidar_pose_i).t = randn(1, 3);
        else
            noise_r = randn(1, 3);
            lidar_pose_struct(lidar_pose_i).R = lidar_pose_struct(lidar_pose_i).R_gt * axang2rotm([noise_r / norm(noise_r), mod(rand(1) * param.randn_pose_noise * pi, pi)]);
            lidar_pose_struct(lidar_pose_i).t = lidar_pose_struct(lidar_pose_i).t_gt + param.randn_pose_noise * randn(1, 3);
        end
    end

    for plane_i = 1:param.plane_num
        if param.randn_plane
            plane_struct.normal_vector_list{1, plane_i} = randn(3, 1);
            plane_struct.q_list{1, plane_i} = randn(3, 1);
        else
            plane_struct.normal_vector_list{1, plane_i} = plane_struct.normal_vector_gt_list{1, plane_i} + randn(3, 1) * param.randn_plane_noise;
            plane_struct.q_list{1, plane_i} = plane_struct.q_gt_list{1, plane_i} + randn(3, 1) * param.randn_plane_noise;
        end

        plane_struct.normal_vector_list{1, plane_i} = plane_struct.normal_vector_list{1, plane_i} / norm(plane_struct.normal_vector_list{1, plane_i});
    end

end
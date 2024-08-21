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

function [plane_struct, lidar_pose_struct, B_sets] = generate_Bset_on_plane(param)

    % -------- pose data generate --------
    lidar_pose_struct = [];

    for lidar_pose_i = 1:param.lidar_pose_num
        if lidar_pose_i == 1
            R = eye(3);
            t = [0, 0, 0];
        else
            r = rand(1, 3) * pi;
            R = axang2rotm([r / norm(r), norm(r)]);
            t = (rand(1, 3) - 0.5) * param.pose_square_size;
        end

        lidar_pose_struct(lidar_pose_i).R_gt = R;
        lidar_pose_struct(lidar_pose_i).t_gt = t;
        
        if(param.randn_pose)
            noise_r = randn(1, 3);
            lidar_pose_struct(lidar_pose_i).R = axang2rotm([noise_r / norm(noise_r), norm(noise_r)]);
            lidar_pose_struct(lidar_pose_i).t = randn(1, 3);
        else
            noise_r = randn(1, 3);
            lidar_pose_struct(lidar_pose_i).R = R * axang2rotm([noise_r / norm(noise_r), param.randn_pose_noise * pi]);
            lidar_pose_struct(lidar_pose_i).t = t + 0.5 * param.pose_square_size * param.randn_pose_noise * randn(1, 3);
        end
    end

    % -------- global point and plane data generate --------
    plane_struct.normal_vector_gt_list = cell(1, param.plane_num);
    plane_struct.q_gt_list = cell(1, param.plane_num);
    plane_struct.normal_vector_list = cell(1, param.plane_num);
    plane_struct.q_list = cell(1, param.plane_num);
    plane_struct_tmp.basevec1 = cell(1, param.plane_num);
    plane_struct_tmp.basevec2 = cell(1, param.plane_num);
    plane_struct_tmp.normal_placement_vector = cell(1, param.plane_num);

    for plane_i = 1:param.plane_num
        basevec1 = rand(3,1) * 2 - 1;
        basevec1 = basevec1 / norm(basevec1);
        basevec2 = rand(3,1) * 2 - 1;
        basevec2 = basevec2 / norm(basevec2);
        normalvec = cross(basevec1, basevec2);
        normalvec = normalvec / norm(normalvec);
        plane_struct_tmp.basevec1{1, plane_i} = basevec1;
        plane_struct_tmp.basevec2{1, plane_i} = basevec2;
        
        % --- get point cloud on each plane ---
        rand_coordinate = (rand(10, 2) - 0.5) * param.plane_square_size;
        rand_point = rand_coordinate * [basevec1'; basevec2'];
        normal_placement_vector = normalvec * (rand() - 0.5) * param.plane_square_size;
        point_cloud_global = rand_point - normal_placement_vector';
        plane_struct_tmp.normal_placement_vector{1, plane_i} = normal_placement_vector;

        % --- get plane parameters ---
        plane_struct.normal_vector_gt_list{1, plane_i} = normalvec;
        plane_struct.q_gt_list{1, plane_i} = mean(point_cloud_global)';
        
        if param.randn_plane
            plane_struct.normal_vector_list{1, plane_i} = randn(3, 1);
            plane_struct.q_list{1, plane_i} = randn(3, 1);
        else
            plane_struct.normal_vector_list{1, plane_i} = plane_struct.normal_vector_gt_list{1, plane_i} + randn(3, 1) * param.randn_plane_noise;
            plane_struct.q_list{1, plane_i} = plane_struct.q_gt_list{1, plane_i} + randn(3, 1) * param.randn_plane_noise;
        end    
    end

    % -------- local point data and id generate --------
    B_sets = cell(param.lidar_pose_num, param.plane_num);

    for lidar_pose_i = 1:param.lidar_pose_num
        R = lidar_pose_struct(lidar_pose_i).R_gt;
        t = lidar_pose_struct(lidar_pose_i).t_gt;
        for plane_i = 1:param.plane_num
            basevec1 = plane_struct_tmp.basevec1{1, plane_i};
            basevec2 = plane_struct_tmp.basevec2{1, plane_i};
            normal_placement_vector = plane_struct_tmp.normal_placement_vector{1, plane_i};
            
            rand_coordinate = (rand(param.point_cloud_num / param.plane_num, 2) - 0.5) * param.plane_square_size;
            rand_point = rand_coordinate * [basevec1'; basevec2'];
            point_cloud_global_gt = rand_point - normal_placement_vector';

            point_cloud_global = point_cloud_global_gt + randn(size(point_cloud_global_gt)) * param.randn_point_noise;
            point_cloud_local = (point_cloud_global - t) * R;
            point_local_homo = [point_cloud_local, ones(size(point_cloud_local,1),1)];
            B_sets{lidar_pose_i, plane_i} = point_local_homo' * point_local_homo;
        end 
    end

end
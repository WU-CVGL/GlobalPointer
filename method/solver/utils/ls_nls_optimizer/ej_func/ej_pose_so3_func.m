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

function [F, J] = ej_pose_so3_func(x, y, point_cloud_cell)
    
    lidar_pose_num = size(point_cloud_cell, 1);
    plane_num = size(point_cloud_cell, 2);
    
    F_cell = cell(lidar_pose_num, plane_num);
    J_cell = cell(lidar_pose_num, plane_num);

    for lidar_pose_i = 1:lidar_pose_num
        pose_id = (lidar_pose_i - 1) * 6;
        r_vec = x(1, pose_id+1:pose_id+3);
        R = axang2rotm([r_vec / norm(r_vec) norm(r_vec)]);
        t = x(1, pose_id+4:pose_id+6);
        
        for plane_i = 1:plane_num
            plane_id = (plane_i - 1) * 4;
            n = y(1, plane_id+1:plane_id+3);
            nq = y(1, plane_id+4);
            
            obs = point_cloud_cell{lidar_pose_i, plane_i};
            F_single = zeros(1, size(obs, 1));
            J_single = zeros(size(obs, 1), lidar_pose_num * 6);

            for obs_i = 1:size(obs, 1)
                Pc = obs(obs_i, :);
                Pg = R * Pc' + t';
      
                F_single(1, obs_i) = n * Pg + nq;
                J_tmp = zeros(1, lidar_pose_num * 6);
                J_tmp(:, pose_id+1:pose_id+3) = - n * skew_symmetric(R * Pc');
                J_tmp(:, pose_id+4:pose_id+6) = n';
                J_tmp(:, plane_id+1:plane_id+4) = [Pg',1];
                J_single(obs_i, :) = J_tmp;
            end

            F_cell{lidar_pose_i, plane_i} = F_single;
            J_cell{lidar_pose_i, plane_i} = J_single;

        end
    end

    F_cell = reshape(F_cell, [1, lidar_pose_num * plane_num]);
    J_cell = reshape(J_cell, [lidar_pose_num * plane_num, 1]);

    F = [cell2mat(F_cell)];
    J = [cell2mat(J_cell)];

end


function S = skew_symmetric(s)

    S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];

end
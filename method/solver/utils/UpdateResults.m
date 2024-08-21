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

function  [lidar_pose_estimate_struct, plane_estimate_struct] = UpdateResults(refined_pose_plane_matlab, lidar_pose_struct, plane_struct, param)

    lidar_pose_estimate_struct = lidar_pose_struct;
    plane_estimate_struct = plane_struct;

    for lidar_pose_i = 1:param.lidar_pose_num
        pose_id = (lidar_pose_i - 1) * 6;
        pose_vec = refined_pose_plane_matlab(1, pose_id+1:pose_id+6);

        if norm(pose_vec(1,1:3)) == 0
            R_near = eye(3);
        else
            R_near = axang2rotm([pose_vec(1, 1:3)/norm(pose_vec(1, 1:3)), norm(pose_vec(1, 1:3))]);
        end
        
        lidar_pose_estimate_struct(lidar_pose_i).R = R_near;
        lidar_pose_estimate_struct(lidar_pose_i).t = pose_vec(1,4:6);
    end

    for plane_i = 1:param.plane_num
        plane_id = param.lidar_pose_num * 6 + (plane_i - 1) * 4;
        plane_estimate_struct.normal_vector_list{1, plane_i} = refined_pose_plane_matlab(1, plane_id+1:plane_id+3)';
        plane_estimate_struct.q_list{1, plane_i} = refined_pose_plane_matlab(1, plane_id+4);
    end

end
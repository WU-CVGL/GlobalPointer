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
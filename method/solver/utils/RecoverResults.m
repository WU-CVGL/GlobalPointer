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
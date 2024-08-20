function lidar_relative_rotation_cell = PrepareRelativeRotation(lidar_pose_struct, param)

    lidar_relative_rotation_cell = cell(param.lidar_pose_num, param.lidar_pose_num);

    for lidar_pose_i = 1:param.lidar_pose_num
        for lidar_pose_j = 1:param.lidar_pose_num

            lidar_relative_rotation_cell{lidar_pose_i, lidar_pose_j} = lidar_pose_struct(lidar_pose_i).R' * lidar_pose_struct(lidar_pose_j).R;
        end
    end



end
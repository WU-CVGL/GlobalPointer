function B_sets = GetBSet(point_cloud_cell, param)


    B_sets = cell(param.lidar_pose_num, param.plane_num);

    for plane_i = 1:param.plane_num
        for lidar_pose_i = 1:param.lidar_pose_num
            point_single = point_cloud_cell{lidar_pose_i, plane_i};
            point_single_homo = [point_single, ones(size(point_single,1),1)];
            B_sets{lidar_pose_i, plane_i} = point_single_homo' * point_single_homo;
        end
    end

end
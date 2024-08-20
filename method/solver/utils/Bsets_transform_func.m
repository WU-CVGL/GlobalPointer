function B_sets_global = Bsets_transform_func(T_cell, B_sets)

    lidar_pose_num = size(B_sets, 1);
    plane_num = size(B_sets, 2);


    B_sets_global = B_sets;
    for lidar_pose_i = 1:lidar_pose_num
        T = T_cell{1, lidar_pose_i};
        for plane_i = 1:plane_num
            B_sets_global{lidar_pose_i, plane_i} = T * B_sets_global{lidar_pose_i, plane_i} * T';
        end
    end

end
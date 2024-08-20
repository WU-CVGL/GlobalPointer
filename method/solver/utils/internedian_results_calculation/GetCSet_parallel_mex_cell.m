function C_sets = GetCSet_parallel_mex_cell(B_sets, lidar_pose_estimate_vec)

    lidar_pose_num = size(B_sets, 1);
    plane_num = size(B_sets, 2);

    C_sets = cell(1, plane_num);

    for plane_i = 1:plane_num
        C = zeros(4, 4);
        for lidar_pose_i = 1:lidar_pose_num
            C = C + lidar_pose_estimate_vec{lidar_pose_i} * B_sets{lidar_pose_i, plane_i} * lidar_pose_estimate_vec{lidar_pose_i}';
        end
        C_sets{1, plane_i} = C;
    end

end
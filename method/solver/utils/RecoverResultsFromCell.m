function lidar_pose_estimate_vec = RecoverResultsFromCell(R_cell, t_cell, param)
    lidar_pose_estimate_cell = cell(1, param.lidar_pose_num);
    for lidar_pose_i = 1:param.lidar_pose_num
        axang = rotm2axang(R_cell{lidar_pose_i, 1});
        lidar_pose_estimate_cell{1, lidar_pose_i} = [axang(1:3) * axang(4), t_cell{lidar_pose_i, 1}'];
    end
    lidar_pose_estimate_vec = cell2mat(lidar_pose_estimate_cell);
    
end
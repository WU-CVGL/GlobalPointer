function lidar_pose_vec = Tcell2Tvec(lidar_pose_cell, param)

    lidar_pose_vec = [];
    for lidar_pose_i = 1:param.lidar_pose_num
        T = lidar_pose_cell{1, lidar_pose_i};
        r = rotm2axang(T(1:3,1:3));
        t = T(1:3,4);
        lidar_pose_vec = [lidar_pose_vec, r(1, 1:3) * r(1,4), t'];
    end

end
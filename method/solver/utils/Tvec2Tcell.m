function lidar_pose_cell = Tvec2Tcell(lidar_pose_vec, param)

    lidar_pose_cell = cell(1, param.lidar_pose_num);
    for lidar_pose_i = 1:param.lidar_pose_num
        pose_id = (lidar_pose_i - 1) * 6;
        r = lidar_pose_vec(1, pose_id+1:pose_id+3);
        t = lidar_pose_vec(1, pose_id+4:pose_id+6);
        if norm(r) == 0
            R = eye(3);
        else
            R = axang2rotm([r / norm(r), norm(r)]);
        end
        T = [R,t';0 0 0 1];
        lidar_pose_cell{1, lidar_pose_i} = T;
    end

end